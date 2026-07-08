/**
 * ESP32 Rover Firmware v2 — Blocking Motor Control
 *
 * Simplified from v5: no non-blocking state machine.
 * Every command writes speed → flush+60ms → direction → done.
 * Guaranteed correct Modbus frame ordering every time.
 *
 * Motor direction (physically confirmed via motor_test v3):
 *   LEFT  motor: UART1/ID=7, CW=forward              → LEFT_MOTOR_INVERTED  = false
 *   RIGHT motor: UART2/ID=2, CCW=forward (inverted)  → RIGHT_MOTOR_INVERTED = true
 */

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point32.h>

// ===========================================================================
// Hardware
// ===========================================================================
#define RX1_PIN          16    // UART1 — Left  motor driver
#define TX1_PIN          17
#define RX2_PIN          14    // UART2 — Right motor driver
#define TX2_PIN          13
#define MODBUS_BAUD_RATE 9600
#define LED_BUILTIN      2

#define LEFT_MOTOR_SLAVE_ID   7
#define RIGHT_MOTOR_SLAVE_ID  2

// ===========================================================================
// Robot constants
// ===========================================================================
const float WHEEL_RADIUS     = 0.05f;
const float WHEEL_SEPARATION = 0.44f;
const float MS_TO_RPM        = 60.0f / (2.0f * PI * WHEEL_RADIUS);
const int   MAX_RPM          = 200;
const int   RPM_DEADZONE     = 5;
const float ENCODER_CPR      = 4000.0f;

// ===========================================================================
// RMCS-2303 Modbus registers
// ===========================================================================
#define REG_SPEED    14
#define REG_CONTROL   2
#define REG_POS_LSB  20    // Actual encoder position, low 16 bits (read-only)
#define REG_POS_MSB  22    // Actual encoder position, high 16 bits (read-only)
#define CTRL_CW      257    // 0x0101 — Mode 1, Enable, CW direction
#define CTRL_CCW     265    // 0x0109 — Mode 1, Enable, CCW direction (inverted bit)
#define CTRL_STOP      0    // 0x0000 — Mode 0, disabled: the actual stop command
                            // NOTE: 0x0100 (256) is "Mode 1 Select" — it does NOT stop a running motor

// ===========================================================================
// Serial ports
// ===========================================================================
HardwareSerial motorSerialL(1);
HardwareSerial motorSerialR(2);

// ===========================================================================
// micro-ROS objects
// ===========================================================================
rclc_executor_t             executor;
rclc_support_t              support;
rcl_allocator_t             allocator;
rcl_node_t                  node;
rcl_subscription_t          subscriber_twist;
geometry_msgs__msg__Twist   twist_msg_buf;
rcl_publisher_t             publisher_ticks;
geometry_msgs__msg__Point32 ticks_msg;
rcl_publisher_t             publisher_enc;
geometry_msgs__msg__Point32 enc_msg;      // Real encoder counts: x=left, y=right, z=fault flags

// ===========================================================================
// Shared state
// ===========================================================================
volatile int  g_left_rpm  = 0;
volatile int  g_right_rpm = 0;
volatile bool g_new_cmd   = false;
unsigned long g_last_cmd_ms = 0;
const unsigned long CMD_WATCHDOG_MS = 1000;

int  cur_left       = 0;
int  cur_right      = 0;
bool motors_stopped = true;   // True once a stop has been sent to the drivers

// Odometry
float         g_left_ticks  = 0.0f;
float         g_right_ticks = 0.0f;
unsigned long last_odom_ms  = 0;
unsigned long last_enc_ms   = 0;

enum AgentState { WAITING_FOR_AGENT, AGENT_CONNECTED };
AgentState agentState = WAITING_FOR_AGENT;

// ===========================================================================
// Send one Modbus frame, flush TX, wait 60ms for ACK round-trip, drain RX.
// This guarantees the motor driver is back in receive mode before next frame.
// ===========================================================================
void mbus_send(HardwareSerial &port, byte slave_id, int reg, unsigned int data) {
    byte sum = slave_id + 0x06 +
               ((reg  >> 8) & 0xFF) + (reg  & 0xFF) +
               ((data >> 8) & 0xFF) + (data & 0xFF);
    byte lrc = (byte)((~sum) + 1);
    char frame[20];
    snprintf(frame, sizeof(frame), ":%02X06%04X%04X%02X\r\n", slave_id, reg, data, lrc);
    port.print(frame);
    port.flush();   // Block until all bytes are transmitted (~18ms at 9600 baud)
    delay(60);      // Wait for driver ACK to complete (17ms TX + margin)
    while (port.available()) port.read();   // Drain ACK bytes
}

// Send to both motors simultaneously (independent UARTs), then wait once for both ACKs.
void mbus_pair(int reg, unsigned int data_l, unsigned int data_r) {
    // Build and queue both frames (both UARTs transmit in parallel)
    auto send_one = [&](HardwareSerial &port, byte slave, unsigned int data) {
        byte sum = slave + 0x06 +
                   ((reg  >> 8) & 0xFF) + (reg  & 0xFF) +
                   ((data >> 8) & 0xFF) + (data & 0xFF);
        byte lrc = (byte)((~sum) + 1);
        char frame[20];
        snprintf(frame, sizeof(frame), ":%02X06%04X%04X%02X\r\n", slave, reg, data, lrc);
        port.print(frame);
    };
    send_one(motorSerialL, LEFT_MOTOR_SLAVE_ID,  data_l);
    send_one(motorSerialR, RIGHT_MOTOR_SLAVE_ID, data_r);
    motorSerialL.flush();   // Wait for UART1 TX done
    motorSerialR.flush();   // Wait for UART2 TX done (already nearly done, parallel)
    delay(60);              // Wait for both ACKs
    while (motorSerialL.available()) motorSerialL.read();
    while (motorSerialR.available()) motorSerialR.read();
}

// ===========================================================================
// Modbus function 03 — read one 16-bit holding register.
// Unlike writes (where the ACK is drained blindly), reads parse and
// LRC-validate the response.
// ===========================================================================

// Queue a read request on a port (does not wait).
static void mbus_read_request(HardwareSerial &port, byte slave, int reg) {
    byte sum = slave + 0x03 +
               ((reg >> 8) & 0xFF) + (reg & 0xFF) + 0x00 + 0x01;
    byte lrc = (byte)((~sum) + 1);
    char frame[20];
    snprintf(frame, sizeof(frame), ":%02X03%04X0001%02X\r\n", slave, reg, lrc);
    while (port.available()) port.read();   // Drop any stale bytes first
    port.print(frame);
}

// Collect and parse the response ":SS 03 02 DDDD LRC\r\n" (13 chars + CRLF).
static bool mbus_read_response(HardwareSerial &port, byte slave,
                               uint16_t &value, unsigned long deadline) {
    char buf[24];
    int  n = 0;
    while ((long)(deadline - millis()) > 0) {
        while (port.available()) {
            char c = port.read();
            if (n < (int)sizeof(buf) - 1) buf[n++] = c;
            if (c != '\n') continue;
            buf[n] = 0;
            if (n < 13 || buf[0] != ':') return false;
            auto hex2 = [&](int i) -> byte {
                char h[3] = {buf[i], buf[i + 1], 0};
                return (byte)strtol(h, NULL, 16);
            };
            byte b_slave = hex2(1), b_func = hex2(3), b_count = hex2(5);
            byte d_hi = hex2(7), d_lo = hex2(9), b_lrc = hex2(11);
            if (b_slave != slave || b_func != 0x03 || b_count != 2) return false;
            byte sum = b_slave + b_func + b_count + d_hi + d_lo;
            if ((byte)((~sum) + 1) != b_lrc) return false;
            value = ((uint16_t)d_hi << 8) | d_lo;
            return true;
        }
    }
    return false;   // Timeout — driver did not answer
}

// Read the same register from both drivers in parallel (~40ms typical).
static void mbus_read_pair(int reg, uint16_t &val_l, uint16_t &val_r,
                           bool &ok_l, bool &ok_r) {
    mbus_read_request(motorSerialL, LEFT_MOTOR_SLAVE_ID,  reg);
    mbus_read_request(motorSerialR, RIGHT_MOTOR_SLAVE_ID, reg);
    motorSerialL.flush();
    motorSerialR.flush();
    unsigned long deadline = millis() + 100;
    ok_l = mbus_read_response(motorSerialL, LEFT_MOTOR_SLAVE_ID,  val_l, deadline);
    ok_r = mbus_read_response(motorSerialR, RIGHT_MOTOR_SLAVE_ID, val_r, deadline);
}

// Direction helper — applies per-motor inversion.
// LEFT  motor: CW=forward  → inverted=false
// RIGHT motor: CCW=forward → inverted=true
static inline unsigned int dir_cmd(int rpm, bool inverted) {
    bool fwd = (rpm > 0);
    if (inverted) fwd = !fwd;
    return fwd ? CTRL_CW : CTRL_CCW;
}

// ===========================================================================
// Stop both motors
// ===========================================================================
void stop_motors() {
    if (motors_stopped) return;  // Already stopped — don't spam the bus
    mbus_pair(REG_SPEED, 0, 0);           // Target speed → 0 (PID ramps down)
    mbus_pair(REG_CONTROL, CTRL_STOP, CTRL_STOP);  // 0x0000 — full disable
    cur_left       = 0;
    cur_right      = 0;
    motors_stopped = true;
}

// ===========================================================================
// Apply motor command — blocking, guaranteed ordering
//
// Normal path (~180ms): speed to both → left direction → right direction.
// Reversal path (~120ms): stop both, reset cur state, return immediately.
//   The next cmd_vel (200ms away at 5Hz) arrives after the motor has had
//   time to coast, finds cur_left/right == 0, takes the normal start path.
//   No long blocking delays — keeping the executor spinning every ~200ms
//   is critical to hold the micro-ROS agent session alive.
// ===========================================================================
void apply_command(int left, int right) {
    if (abs(left)  < RPM_DEADZONE) left  = 0;
    if (abs(right) < RPM_DEADZONE) right = 0;

    if (left == 0 && right == 0) {
        stop_motors();
        return;
    }

    // Same command as last time — drivers already latched it, nothing to do.
    // Without this guard, teleop's continuous 5Hz stream re-sends identical
    // frames every 200ms and the 180ms Modbus block starves the executor.
    if (!motors_stopped && left == cur_left && right == cur_right) return;

    bool left_flip  = (left  != 0 && cur_left  != 0 && (left  > 0) != (cur_left  > 0));
    bool right_flip = (right != 0 && cur_right != 0 && (right > 0) != (cur_right > 0));

    if (left_flip || right_flip) {
        mbus_pair(REG_SPEED, 0, 0);
        mbus_pair(REG_CONTROL, CTRL_STOP, CTRL_STOP);  // 0x0000 — actual stop
        cur_left       = 0;
        cur_right      = 0;
        motors_stopped = true;
        return;  // Next cmd_vel starts fresh — no blocking wait needed
    }

    mbus_pair(REG_SPEED,
              (left  != 0) ? (uint16_t)abs(left)  : 0,
              (right != 0) ? (uint16_t)abs(right) : 0);

    // Stagger L then R by one mbus round-trip (~60ms) to avoid simultaneous
    // startup inrush on both drivers at the same instant.
    mbus_send(motorSerialL, LEFT_MOTOR_SLAVE_ID,  REG_CONTROL,
              (left  != 0) ? dir_cmd(left,  false) : CTRL_STOP);
    mbus_send(motorSerialR, RIGHT_MOTOR_SLAVE_ID, REG_CONTROL,
              (right != 0) ? dir_cmd(right, true)  : CTRL_STOP);

    cur_left       = left;
    cur_right      = right;
    motors_stopped = false;
}

// ===========================================================================
// /cmd_vel callback
// ===========================================================================
void twist_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    float lin = msg->linear.x;
    float ang = msg->angular.z;
    float left_ms  = lin - (ang * WHEEL_SEPARATION / 2.0f);
    float right_ms = lin + (ang * WHEEL_SEPARATION / 2.0f);
    g_left_rpm  = constrain((int)(left_ms  * MS_TO_RPM), -MAX_RPM, MAX_RPM);
    g_right_rpm = constrain((int)(right_ms * MS_TO_RPM), -MAX_RPM, MAX_RPM);
    g_new_cmd   = true;
}

// ===========================================================================
// micro-ROS entity lifecycle
// ===========================================================================
bool create_entities() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "esp32_rover_v2", "", &support) != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &subscriber_twist, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &publisher_ticks, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
            "/wheel_ticks") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &publisher_enc, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
            "/encoder_ticks") != RCL_RET_OK) return false;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(
            &executor, &subscriber_twist, &twist_msg_buf,
            &twist_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
    rcl_publisher_fini(&publisher_enc, &node);
    rcl_publisher_fini(&publisher_ticks, &node);
    rcl_subscription_fini(&subscriber_twist, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ===========================================================================
// Setup
// ===========================================================================
void setup() {
    Serial.begin(115200);
    // NOTE: UART0 is shared with micro-ROS. No Serial.print before set_microros_transports().
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    motorSerialL.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX1_PIN, TX1_PIN);
    motorSerialR.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
    delay(100);

    // Force a stop at boot — drivers may still be running from before an
    // ESP32 reset, so bypass the motors_stopped guard.
    motors_stopped = false;
    stop_motors();
    delay(200);

    set_microros_transports();
}

// ===========================================================================
// Main loop
// ===========================================================================
void loop() {
    switch (agentState) {

    case WAITING_FOR_AGENT:
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            if (create_entities()) {
                agentState    = AGENT_CONNECTED;
                g_last_cmd_ms = millis();
                last_odom_ms  = millis();
                digitalWrite(LED_BUILTIN, HIGH);
            } else {
                destroy_entities();
            }
        }
        break;

    case AGENT_CONNECTED:
        // Spin executor — delivers /cmd_vel messages to twist_callback
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

        // Apply command if one arrived (blocking — 30-60ms max)
        if (g_new_cmd) {
            g_new_cmd     = false;
            g_last_cmd_ms = millis();
            apply_command(g_left_rpm, g_right_rpm);
        }

        // Watchdog — stop if no command for 1 second
        if (g_last_cmd_ms > 0 && (millis() - g_last_cmd_ms) > CMD_WATCHDOG_MS) {
            stop_motors();
            g_last_cmd_ms = 0;
        }

        // Odometry at 20Hz (dead-reckoning from commanded RPM)
        if (millis() - last_odom_ms >= 50) {
            float dt = (millis() - last_odom_ms) / 1000.0f;
            last_odom_ms = millis();

            g_left_ticks  += (cur_left  / 60.0f) * ENCODER_CPR * dt;
            g_right_ticks += (cur_right / 60.0f) * ENCODER_CPR * dt;

            ticks_msg.x = g_left_ticks;
            ticks_msg.y = g_right_ticks;
            ticks_msg.z = 0.0f;
            rcl_publish(&publisher_ticks, &ticks_msg, NULL);
        }

        // Real encoder feedback at 2Hz → /encoder_ticks
        // x = left raw 32-bit position, y = right raw position,
        // z = fault flags (0 = both OK, +1 left read failed, +2 right failed;
        //     on failure the last good value is republished).
        // ~80-100ms of bus time per poll; commands still take priority since
        // g_new_cmd is checked first each loop pass.
        if (millis() - last_enc_ms >= 500) {
            last_enc_ms = millis();
            uint16_t l_lsb = 0, l_msb = 0, r_lsb = 0, r_msb = 0;
            bool okl1, okr1, okl2, okr2;
            mbus_read_pair(REG_POS_LSB, l_lsb, r_lsb, okl1, okr1);
            mbus_read_pair(REG_POS_MSB, l_msb, r_msb, okl2, okr2);
            bool ok_l = okl1 && okl2;
            bool ok_r = okr1 && okr2;
            if (ok_l) enc_msg.x = (float)(int32_t)(((uint32_t)l_msb << 16) | l_lsb);
            if (ok_r) enc_msg.y = (float)(int32_t)(((uint32_t)r_msb << 16) | r_lsb);
            enc_msg.z = (ok_l ? 0.0f : 1.0f) + (ok_r ? 0.0f : 2.0f);
            rcl_publish(&publisher_enc, &enc_msg, NULL);
        }

        // Check agent still alive — every 2s, NOT every loop pass.
        // Pinging every iteration floods UART0 and competes with incoming
        // /cmd_vel traffic on the same serial transport.
        {
            static unsigned long last_ping_ms = 0;
            if (millis() - last_ping_ms >= 2000) {
                last_ping_ms = millis();
                if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) {
                    stop_motors();
                    destroy_entities();
                    agentState = WAITING_FOR_AGENT;
                    digitalWrite(LED_BUILTIN, LOW);
                }
            }
        }
        break;
    }
}
