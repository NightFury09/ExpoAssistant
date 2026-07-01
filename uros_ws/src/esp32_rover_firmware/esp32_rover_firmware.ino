/**
 * esp32_rover_firmware.ino  —  v3 (Direct Modbus Write)
 *
 * KEY FIX vs v2:
 *   The RMCS2303 library hides a 5ms delay + 30ms ACK-wait inside every
 *   WriteSingleRegister() call.  4 calls per cmd_vel = ~140-200ms block.
 *   teleop sends at 10Hz = 100ms interval → ESP32 always executes STALE
 *   commands → random / mixed motor directions.
 *
 *   Fix: bypass the library entirely for motor commands. Use a raw Modbus
 *   ASCII write (fire-and-forget, no ACK wait). The RMCS-2303 acts on the
 *   frame the instant it finishes receiving it. ACK reading is optional.
 *
 *   Result: total block per cmd_vel drops from ~200ms → ~20ms.
 *
 * Hardware confirmed (2026-05-29):
 *   LEFT_MOTOR_SLAVE_ID  = 7  (physical jumpers: JP1+JP2+JP3 closed)
 *   RIGHT_MOTOR_SLAVE_ID = 2  (physical jumpers: JP2 closed)
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// ---------------------------------------------------------------------------
// Pin & Hardware Configuration
// ---------------------------------------------------------------------------
#define RX1_PIN          16   // UART1 RX — Left motor driver TX
#define TX1_PIN          17   // UART1 TX — Left motor driver RX
#define RX2_PIN          14   // UART2 RX — Right motor driver TX
#define TX2_PIN          13   // UART2 TX — Right motor driver RX
#define MODBUS_BAUD_RATE 9600
#define LED_BUILTIN      2

#define LEFT_MOTOR_SLAVE_ID  7
#define RIGHT_MOTOR_SLAVE_ID 2

// ---------------------------------------------------------------------------
// Robot Physical Constants
// ---------------------------------------------------------------------------
const float WHEEL_RADIUS     = 0.05f;   // metres
const float WHEEL_SEPARATION = 0.44f;   // metres (centre-to-centre)
const float MS_TO_RPM        = 60.0f / (2.0f * PI * WHEEL_RADIUS);
const int   MAX_RPM          = 200;
const int   RPM_DEADZONE     = 5;

// ---------------------------------------------------------------------------
// Motor direction convention — SET BY CALIBRATION
// If the left motor physically spins backward when commanded forward,
// set LEFT_MOTOR_INVERTED to true.
// Run motor_calibration.ino first to determine the correct value.
// ---------------------------------------------------------------------------
const bool LEFT_MOTOR_INVERTED  = false;   // true = flip left motor direction
const bool RIGHT_MOTOR_INVERTED = false;  // typically false

// ---------------------------------------------------------------------------
// RMCS-2303 Modbus Register Map
// ---------------------------------------------------------------------------
#define REG_CONTROL  2    // Control register (enable, disable, direction)
#define REG_SPEED    14   // Speed setpoint register (RPM)

// Control register values (from RMCS-2303 manual):
//   257 (0x0101) = Digital Speed Mode, CW
//   265 (0x0109) = Digital Speed Mode, CCW
//   256 (0x0100) = Digital Speed Mode, disabled (CW reference)
#define CTRL_CW      257
#define CTRL_CCW     265
#define CTRL_DISABLE 256

// ---------------------------------------------------------------------------
// Serial ports
// ---------------------------------------------------------------------------
HardwareSerial motorSerialL(1);   // UART1 — Left motor
HardwareSerial motorSerialR(2);   // UART2 — Right motor

// ---------------------------------------------------------------------------
// micro-ROS Objects
// ---------------------------------------------------------------------------
rclc_executor_t           executor;
rclc_support_t            support;
rcl_allocator_t           allocator;
rcl_node_t                node;
rcl_subscription_t        subscriber_twist;
geometry_msgs__msg__Twist twist_msg_buf;

// ---------------------------------------------------------------------------
// Shared state — written by callback, consumed by loop()
// ---------------------------------------------------------------------------
volatile int  g_left_rpm  = 0;
volatile int  g_right_rpm = 0;
volatile bool g_new_cmd   = false;
unsigned long g_last_cmd_ms = 0;
const unsigned long CMD_WATCHDOG_MS = 500;

// ---------------------------------------------------------------------------
// Reconnection State Machine
// ---------------------------------------------------------------------------
enum AgentState { WAITING_FOR_AGENT, AGENT_CONNECTED };
AgentState agentState = WAITING_FOR_AGENT;

// ===========================================================================
// DIRECT MODBUS ASCII WRITE — No ACK wait, no hidden delays.
//
// Constructs and sends a Modbus ASCII Write Single Register frame:
//   :SS 06 AH AL DH DL LRC \r\n
// where SS=slave, AH/AL=register address, DH/DL=data, LRC=checksum.
//
// The RMCS-2303 begins executing the command as soon as it receives \r\n.
// We do NOT wait for its response — fire and forget.
// ===========================================================================
void modbus_write(HardwareSerial &port, byte slave_id,
                  int address, unsigned int data) {
    byte AddrHi = (address >> 8) & 0xFF;
    byte AddrLo =  address       & 0xFF;
    byte DataHi = (data   >> 8) & 0xFF;
    byte DataLo =  data         & 0xFF;
    const byte FC = 0x06;  // Write Single Register

    // Modbus ASCII LRC = two's complement of the sum of all payload bytes
    byte sum = slave_id + FC + AddrHi + AddrLo + DataHi + DataLo;
    byte lrc = (byte)((~sum) + 1);

    // Build ASCII frame into a small stack buffer
    char frame[20];
    snprintf(frame, sizeof(frame), ":%02X%02X%02X%02X%02X%02X%02X\r\n",
             slave_id, FC, AddrHi, AddrLo, DataHi, DataLo, lrc);

    // Write to UART TX buffer — returns immediately.
    // Hardware UART sends the bytes asynchronously in background.
    port.print(frame);
}

// ===========================================================================
// COMMAND BOTH MOTORS — parallel, low-latency
//
// Total blocking time: ~20ms (one delay for both motors simultaneously)
// vs the old library approach: ~200ms (sequential per-motor ACK waits)
//
// Sequence:
//   1. Send Speed frame to LEFT  (UART1 starts transmitting in background)
//   2. Send Speed frame to RIGHT (UART2 starts transmitting in background)
//      ↑ Both UARTs transmit simultaneously since they are separate hardware
//   3. delay(20) — gives both drivers time to receive and latch the speed
//   4. Send Enable+Direction frame to LEFT
//   5. Send Enable+Direction frame to RIGHT
//      ↑ Again both transmit simultaneously
// ===========================================================================
void command_motors(int left_rpm, int right_rpm) {
    // Apply dead-zone
    if (abs(left_rpm)  < RPM_DEADZONE) left_rpm  = 0;
    if (abs(right_rpm) < RPM_DEADZONE) right_rpm = 0;

    // --- PHASE 1: Set speed (or disable) on both motors simultaneously ---
    if (left_rpm == 0) {
        modbus_write(motorSerialL, LEFT_MOTOR_SLAVE_ID,  REG_CONTROL, CTRL_DISABLE);
    } else {
        modbus_write(motorSerialL, LEFT_MOTOR_SLAVE_ID,  REG_SPEED, (uint16_t)abs(left_rpm));
    }

    if (right_rpm == 0) {
        modbus_write(motorSerialR, RIGHT_MOTOR_SLAVE_ID, REG_CONTROL, CTRL_DISABLE);
    } else {
        modbus_write(motorSerialR, RIGHT_MOTOR_SLAVE_ID, REG_SPEED, (uint16_t)abs(right_rpm));
    }

    // If both stopped — done, no need to send enable frames
    if (left_rpm == 0 && right_rpm == 0) return;

    // Wait for speed frames to propagate over UART and be latched by the drivers.
    // At 9600 baud, a 17-byte Modbus ASCII frame takes ~17.7ms to transmit.
    delay(20);

    // --- PHASE 2: Enable with direction on both motors simultaneously ---
    if (left_rpm != 0) {
        bool fwd = (left_rpm > 0);
        // Apply physical mounting inversion for left motor
        if (LEFT_MOTOR_INVERTED) fwd = !fwd;
        modbus_write(motorSerialL, LEFT_MOTOR_SLAVE_ID,
                     REG_CONTROL, fwd ? CTRL_CW : CTRL_CCW);
    }

    if (right_rpm != 0) {
        bool fwd = (right_rpm > 0);
        if (RIGHT_MOTOR_INVERTED) fwd = !fwd;
        modbus_write(motorSerialR, RIGHT_MOTOR_SLAVE_ID,
                     REG_CONTROL, fwd ? CTRL_CW : CTRL_CCW);
    }
}

// Hard-stop both motors immediately (used on watchdog / disconnect)
void stop_all_motors() {
    modbus_write(motorSerialL, LEFT_MOTOR_SLAVE_ID,  REG_CONTROL, CTRL_DISABLE);
    modbus_write(motorSerialR, RIGHT_MOTOR_SLAVE_ID, REG_CONTROL, CTRL_DISABLE);
}

// ---------------------------------------------------------------------------
// /cmd_vel Callback — store values only, no blocking calls here
// ---------------------------------------------------------------------------
void twist_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;

    float lin = msg->linear.x;
    float ang = msg->angular.z;

    // Differential drive kinematics
    float left_ms  = lin - (ang * WHEEL_SEPARATION / 2.0f);
    float right_ms = lin + (ang * WHEEL_SEPARATION / 2.0f);

    int l = constrain((int)(left_ms  * MS_TO_RPM), -MAX_RPM, MAX_RPM);
    int r = constrain((int)(right_ms * MS_TO_RPM), -MAX_RPM, MAX_RPM);

    g_left_rpm  = l;
    g_right_rpm = r;
    g_new_cmd   = true;
}

// ---------------------------------------------------------------------------
// Create / Destroy micro-ROS entities
// ---------------------------------------------------------------------------
bool create_entities() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "esp32_rover_controller", "", &support) != RCL_RET_OK) return false;
    if (rclc_subscription_init_default(
            &subscriber_twist, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel") != RCL_RET_OK) return false;
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(
            &executor, &subscriber_twist, &twist_msg_buf,
            &twist_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
    rcl_subscription_fini(&subscriber_twist, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Initialise UART ports for Modbus communication
    motorSerialL.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX1_PIN, TX1_PIN);
    motorSerialR.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
    delay(100);  // Wait for UART to stabilise

    // Safe boot state — stop both motors
    stop_all_motors();
    delay(200);

    Serial.println("ESP32 Rover Firmware v3 (Direct Modbus)");
    Serial.print("Left  Slave ID: "); Serial.println(LEFT_MOTOR_SLAVE_ID);
    Serial.print("Right Slave ID: "); Serial.println(RIGHT_MOTOR_SLAVE_ID);
    Serial.println("Waiting for micro-ROS agent...");

    set_microros_transports();
}

// ---------------------------------------------------------------------------
// Main Loop — State Machine
// ---------------------------------------------------------------------------
void loop() {
    switch (agentState) {

    case WAITING_FOR_AGENT:
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            if (create_entities()) {
                agentState    = AGENT_CONNECTED;
                g_last_cmd_ms = millis();
                digitalWrite(LED_BUILTIN, HIGH);
                Serial.println("Agent connected. Ready for /cmd_vel.");
            } else {
                destroy_entities();
            }
        }
        break;

    case AGENT_CONNECTED:
        if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) {
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                stop_all_motors();
                g_new_cmd = false;
                destroy_entities();
                agentState = WAITING_FOR_AGENT;
                digitalWrite(LED_BUILTIN, LOW);
                Serial.println("Agent lost. Motors stopped. Reconnecting...");
            }
            break;
        }

        // Apply new motor command if one arrived
        if (g_new_cmd) {
            g_new_cmd     = false;
            g_last_cmd_ms = millis();
            command_motors(g_left_rpm, g_right_rpm);
        }

        // Watchdog — stop if no command for CMD_WATCHDOG_MS
        if (g_last_cmd_ms > 0 && (millis() - g_last_cmd_ms) > CMD_WATCHDOG_MS) {
            stop_all_motors();
            g_last_cmd_ms = 0;
        }
        break;
    }
}
