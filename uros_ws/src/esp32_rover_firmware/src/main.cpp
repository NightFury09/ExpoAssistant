/**
 * main.cpp  —  ESP32 Rover Firmware v5 (Clean State Machine)
 *
 * v5: Complete rewrite of motor command logic.
 *     - DISABLE only on stop or direction reversal (never on repeated same-direction commands)
 *     - Clean 2-phase state machine: SPEED write → 20ms → DIRECTION write
 *     - No duplicate filter (motor driver handles idempotent commands gracefully)
 *     - Dead-reckoning odometry published at 20Hz
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
// Pin & Hardware Configuration
// ===========================================================================
#define RX1_PIN          16
#define TX1_PIN          17
#define RX2_PIN          14
#define TX2_PIN          13
#define MODBUS_BAUD_RATE 9600
#define LED_BUILTIN      2

#define LEFT_MOTOR_SLAVE_ID  7
#define RIGHT_MOTOR_SLAVE_ID 2

// ===========================================================================
// Robot Physical Constants
// ===========================================================================
const float WHEEL_RADIUS     = 0.05f;
const float WHEEL_SEPARATION = 0.44f;
const float MS_TO_RPM        = 60.0f / (2.0f * PI * WHEEL_RADIUS);
const int   MAX_RPM          = 200;
const int   RPM_DEADZONE     = 5;

// ===========================================================================
// Direction Calibration — flip these if a motor runs backwards
// ===========================================================================
const bool LEFT_MOTOR_INVERTED  = true;
const bool RIGHT_MOTOR_INVERTED = false;

// ===========================================================================
// RMCS-2303 Register Map
// ===========================================================================
#define REG_SPEED    14
#define REG_CONTROL   2

#define CTRL_CW      257   // Enable + Clockwise
#define CTRL_CCW     265   // Enable + Counter-clockwise
#define CTRL_DISABLE 256   // Disable (motor free-wheels)

// ===========================================================================
// Serial Ports
// ===========================================================================
HardwareSerial motorSerialL(1);   // Left motor  — UART1
HardwareSerial motorSerialR(2);   // Right motor — UART2

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

// ===========================================================================
// Shared command state (written by callback, read by loop)
// ===========================================================================
volatile int  g_left_rpm  = 0;
volatile int  g_right_rpm = 0;
volatile bool g_new_cmd   = false;
unsigned long g_last_cmd_ms = 0;
const unsigned long CMD_WATCHDOG_MS = 1000;  // 1s — generous for 10Hz teleop

// ===========================================================================
// Motor command state machine
// ===========================================================================
enum McState { MC_IDLE, MC_WAIT_DISABLE, MC_WRITE_SPEED, MC_WAIT_SPEED, MC_WRITE_DIR };
McState       mc_state    = MC_IDLE;
unsigned long mc_timer    = 0;
int           mc_left     = 0;   // target left RPM (with sign)
int           mc_right    = 0;   // target right RPM (with sign)
int           cur_left    = 0;   // what is currently running
int           cur_right   = 0;

// ===========================================================================
// Dead-reckoning odometry
// ===========================================================================
const float   ENCODER_CPR  = 4000.0f;
float         g_left_ticks  = 0.0f;
float         g_right_ticks = 0.0f;
unsigned long last_odom_ms  = 0;

// ===========================================================================
// Agent state machine
// ===========================================================================
enum AgentState { WAITING_FOR_AGENT, AGENT_CONNECTED };
AgentState agentState = WAITING_FOR_AGENT;

// ===========================================================================
// Helpers
// ===========================================================================
static inline int sign(int v) { return (v > 0) ? 1 : (v < 0) ? -1 : 0; }

void modbus_write(HardwareSerial &port, byte slave_id, int address, unsigned int data) {
    byte sum = slave_id + 0x06 +
               ((address >> 8) & 0xFF) + (address & 0xFF) +
               ((data    >> 8) & 0xFF) + (data    & 0xFF);
    byte lrc = (byte)((~sum) + 1);
    char frame[20];
    snprintf(frame, sizeof(frame), ":%02X06%04X%04X%02X\r\n",
             slave_id, address, data, lrc);
    port.print(frame);
}

void write_direction(HardwareSerial &port, byte slave_id, int rpm, bool inverted) {
    bool fwd = (rpm > 0);
    if (inverted) fwd = !fwd;
    modbus_write(port, slave_id, REG_CONTROL, fwd ? CTRL_CW : CTRL_CCW);
}

// ===========================================================================
// Motor stop — immediate, bypasses state machine
// ===========================================================================
void stop_motors_immediate() {
    modbus_write(motorSerialL, LEFT_MOTOR_SLAVE_ID,  REG_CONTROL, CTRL_DISABLE);
    modbus_write(motorSerialR, RIGHT_MOTOR_SLAVE_ID, REG_CONTROL, CTRL_DISABLE);
    mc_left    = 0;
    mc_right   = 0;
    cur_left   = 0;
    cur_right  = 0;
    g_left_rpm = 0;
    g_right_rpm= 0;
    mc_state   = MC_IDLE;
}

// ===========================================================================
// command_motors — called from loop() when g_new_cmd is set
// ===========================================================================
void command_motors(int left, int right) {
    // Apply deadzone
    if (abs(left)  < RPM_DEADZONE) left  = 0;
    if (abs(right) < RPM_DEADZONE) right = 0;

    // Immediate stop
    if (left == 0 && right == 0) {
        stop_motors_immediate();
        return;
    }

    mc_left  = left;
    mc_right = right;

    // Check if direction reversal is needed for either motor
    bool dir_change = (left  != 0 && cur_left  != 0 && sign(left)  != sign(cur_left))  ||
                      (right != 0 && cur_right != 0 && sign(right) != sign(cur_right));

    if (dir_change) {
        modbus_write(motorSerialL, LEFT_MOTOR_SLAVE_ID,  REG_CONTROL, CTRL_DISABLE);
        modbus_write(motorSerialR, RIGHT_MOTOR_SLAVE_ID, REG_CONTROL, CTRL_DISABLE);
        mc_timer = millis();
        mc_state = MC_WAIT_DISABLE;
    } else {
        mc_timer = millis();
        mc_state = MC_WRITE_SPEED;
    }
}

// ===========================================================================
// Non-blocking state machine — called every loop iteration
// ===========================================================================
void process_motor_commands() {
    switch (mc_state) {

    case MC_IDLE:
        break;

    case MC_WAIT_DISABLE:
        if (millis() - mc_timer >= 20) {
            mc_state = MC_WRITE_SPEED;
        }
        break;

    case MC_WRITE_SPEED:
        if (mc_left  != 0) modbus_write(motorSerialL, LEFT_MOTOR_SLAVE_ID,  REG_SPEED, (uint16_t)abs(mc_left));
        if (mc_right != 0) modbus_write(motorSerialR, RIGHT_MOTOR_SLAVE_ID, REG_SPEED, (uint16_t)abs(mc_right));
        mc_timer = millis();
        mc_state = MC_WAIT_SPEED;
        break;

    case MC_WAIT_SPEED:
        if (millis() - mc_timer >= 20) {
            mc_state = MC_WRITE_DIR;
        }
        break;

    case MC_WRITE_DIR:
        if (mc_left  != 0) write_direction(motorSerialL, LEFT_MOTOR_SLAVE_ID,  mc_left,  LEFT_MOTOR_INVERTED);
        if (mc_right != 0) write_direction(motorSerialR, RIGHT_MOTOR_SLAVE_ID, mc_right, RIGHT_MOTOR_INVERTED);
        cur_left  = mc_left;
        cur_right = mc_right;
        mc_state  = MC_IDLE;
        break;
    }
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
// micro-ROS entity management
// ===========================================================================
bool create_entities() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "esp32_rover_controller", "", &support) != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &subscriber_twist, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &publisher_ticks, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
            "/wheel_ticks") != RCL_RET_OK) return false;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(
            &executor, &subscriber_twist, &twist_msg_buf,
            &twist_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
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
    // NOTE: Serial (UART0) is shared with micro-ROS transport.
    // Do NOT print anything here — output would corrupt the micro-ROS session.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    motorSerialL.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX1_PIN, TX1_PIN);
    motorSerialR.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
    delay(100);

    stop_motors_immediate();
    delay(200);

    set_microros_transports();
}

// ===========================================================================
// Main Loop
// ===========================================================================
void loop() {
    switch (agentState) {

    case WAITING_FOR_AGENT:
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            if (create_entities()) {
                agentState    = AGENT_CONNECTED;
                g_last_cmd_ms = millis();
                last_odom_ms  = millis();
                digitalWrite(LED_BUILTIN, HIGH);   // LED ON = agent connected
            } else {
                destroy_entities();
            }
        }
        break;

    case AGENT_CONNECTED:
        // Spin ROS executor — calls twist_callback if new message
        if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)) != RCL_RET_OK) {
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                stop_motors_immediate();
                destroy_entities();
                agentState = WAITING_FOR_AGENT;
                digitalWrite(LED_BUILTIN, LOW);    // LED OFF = agent lost
            }
            break;
        }

        // Process new velocity command
        if (g_new_cmd) {
            g_new_cmd     = false;
            g_last_cmd_ms = millis();
            command_motors(g_left_rpm, g_right_rpm);
        }

        // Run state machine (non-blocking Modbus writes)
        process_motor_commands();

        // Watchdog — stop motors if no command for 500ms
        if (g_last_cmd_ms > 0 && (millis() - g_last_cmd_ms) > CMD_WATCHDOG_MS) {
            stop_motors_immediate();
            g_last_cmd_ms = 0;
        }

        // Odometry at 20Hz
        if (millis() - last_odom_ms >= 50) {
            unsigned long now = millis();
            float dt = (now - last_odom_ms) / 1000.0f;
            last_odom_ms = now;

            // Flush any motor driver response bytes from RX buffers
            while (motorSerialL.available()) motorSerialL.read();
            while (motorSerialR.available()) motorSerialR.read();

            // Dead-reckoning integration using current commanded RPM
            g_left_ticks  += (cur_left  / 60.0f) * ENCODER_CPR * dt;
            g_right_ticks += (cur_right / 60.0f) * ENCODER_CPR * dt;

            ticks_msg.x = g_left_ticks;
            ticks_msg.y = g_right_ticks;
            ticks_msg.z = 0.0f;
            rcl_publish(&publisher_ticks, &ticks_msg, NULL);
        }

        break;
    }
}
