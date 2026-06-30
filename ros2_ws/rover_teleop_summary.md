# Context: ESP32 Rover Firmware & RMCS-2303 Motor Driver Configuration

This document serves as the contextual reference for the ESP32 rover firmware and motor driver settings. It details the actual hardware integration, Modbus addressing, wiring, and micro-ROS firmware state machine used on the differential-drive rover.

---

## 1. Hardware Architecture & Component Stack

* **Brain:** ESP32 Development Module
* **Motor Drivers:** Two RMCS-2303 DC Servo Controllers (Jumper-Based Configuration Model)
* **Motors:** Two RMCS-5012 DC Servo Motors with integrated Quadrature Encoders (connected to the drivers)
* **Power Supply:** 12V High-Current Supply (shared across drivers and motors)
* **Communication Interface:** Hardware UART Serial over Modbus RTU ASCII protocol (9600 Baud, 8N1)
* **Network Layer:** micro-ROS (ROS 2 Humble) via USB-Serial transport layer on UART0 (115200 Baud)

---

## 2. Hardware Wiring Configuration Matrix

The ESP32 firmware communicates with the Left and Right motor controllers using two independent hardware serial buses (`Serial1` and `Serial2`) to isolate logic and commands:

| Component Subsystem | ESP32 GPIO Pin | Driver Terminal Label | Connection Role |
| :--- | :--- | :--- | :--- |
| **Left Motor (Serial1)** | **GPIO 17** | **RX** | ESP32 TX $\rightarrow$ Driver RX |
| **Left Motor (Serial1)** | **GPIO 16** | **TX** | ESP32 RX $\rightarrow$ Driver TX |
| **Left Motor Ground** | **GND** | **GND** | Logic Common Ground Reference |
| **Right Motor (Serial2)**| **GPIO 13** | **RX** | ESP32 TX $\rightarrow$ Driver RX |
| **Right Motor (Serial2)**| **GPIO 14** | **TX** | ESP32 RX $\rightarrow$ Driver TX |
| **Right Motor Ground** | **GND** | **GND** | Logic Common Ground Reference |

---

## 3. Hardware Jumper Config & Slave ID Addressing

The RMCS-2303 reads jumper-based binary address configurations during boot initialization. The physical jumpers must correspond to the binary representation of the Slave IDs defined in software:

### Binary Address Assignments

| Target Subsystem | Software ID | JP1 (Bit 0) | JP2 (Bit 1) | JP3 (Bit 2) | Physical Jumper State |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Left Motor Driver** | `LEFT_MOTOR_SLAVE_ID 7` | **CLOSED** | **CLOSED** | **CLOSED** | All three address jumpers closed |
| **Right Motor Driver**| `RIGHT_MOTOR_SLAVE_ID 2`| OPEN | **CLOSED** | OPEN | Only the second jumper (JP2) closed |

* **UART/Modbus Selection (JP4 - JP6):** Leave these jumpers **OPEN** to select Modbus digital serial mode at the default **9600 bps (8N1)** configuration.

---

## 4. Modbus ASCII Register Map

The firmware directly implements Modbus ASCII protocol commands to write to the driver registers, avoiding blocking library functions:

* **Register Address `14` (REG_SPEED):** Target speed register (unsigned 16-bit integer, absolute RPM).
* **Register Address `2` (REG_CONTROL):** Target control and direction register:
  * `CTRL_CW  (257)`: Enable motor + Clockwise rotation
  * `CTRL_CCW (265)`: Enable motor + Counter-clockwise rotation
  * `CTRL_DISABLE (256)`: Disable motor (free-wheels)

---

## 5. Non-Blocking Control State Machine & Odometry

The firmware implements a non-blocking state machine to manage Modbus commands. It prevents execution freezes during the 20ms write cycles required by the RMCS-2303 controllers:

1. **Non-blocking State Transitions:** Command updates transit through states (`MC_WRITE_SPEED` $\rightarrow$ 20ms delay $\rightarrow$ `MC_WRITE_DIR`) cooperatively using `millis()`.
2. **Direction Reversal Handling:** If a direction reversal is detected (e.g., forward to backward), the motors are disabled first, allowed to settle for 20ms, and then speed and direction updates are sent.
3. **Safety Watchdog:** If no `/cmd_vel` message is received within `1.0` seconds (`CMD_WATCHDOG_MS = 1000`), the watchdog immediately stops the motors.
4. **Dead-Reckoning Odometry:** Because physical encoders are connected to the drivers but not routed back to the ESP32 pins, the ESP32 firmware integrates the commanded wheel RPM to simulate encoder tick counts at 20Hz:
   $$\text{Ticks} \mathrel{+}= \left(\frac{\text{Commanded RPM}}{60}\right) \times \text{CPR} \times dt$$
   The resulting ticks are published on the `/wheel_ticks` topic (`geometry_msgs/Point32`), which the ROS 2 workspace node `rover_odometry` subscribes to for computing the robot position (`/odom`).

---

## 6. Physical and Kinematic Parameters

The following parameters are hardcoded and shared between the firmware, URDF description, and ROS 2 odometry configurations:

* **Wheel Radius:** `0.05 m` (50 mm)
* **Wheel Separation (Track Width):** `0.44 m` (440 mm center-to-center)
* **Encoder Counts Per Revolution (CPR):** `4000.0`
* **Max Speed Limit:** Clamped at `200 RPM` to protect the physical system.

---

## 7. Firmware Reference Implementation

The active code deployed to the ESP32 (located at `uros_ws/src/esp32_rover_firmware/src/main.cpp`) is structured as follows:

```cpp
#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point32.h>

#define RX1_PIN          16
#define TX1_PIN          17
#define RX2_PIN          14
#define TX2_PIN          13
#define MODBUS_BAUD_RATE 9600
#define LED_BUILTIN      2

#define LEFT_MOTOR_SLAVE_ID  7
#define RIGHT_MOTOR_SLAVE_ID 2

const float WHEEL_RADIUS     = 0.05f;
const float WHEEL_SEPARATION = 0.44f;
const float MS_TO_RPM        = 60.0f / (2.0f * PI * WHEEL_RADIUS);
const int   MAX_RPM          = 200;
const int   RPM_DEADZONE     = 5;

const bool LEFT_MOTOR_INVERTED  = true;
const bool RIGHT_MOTOR_INVERTED = false;

#define REG_SPEED    14
#define REG_CONTROL   2
#define CTRL_CW      257
#define CTRL_CCW     265
#define CTRL_DISABLE 256

HardwareSerial motorSerialL(1);
HardwareSerial motorSerialR(2);

rclc_executor_t             executor;
rclc_support_t              support;
rcl_allocator_t             allocator;
rcl_node_t                  node;
rcl_subscription_t          subscriber_twist;
geometry_msgs__msg__Twist   twist_msg_buf;
rcl_publisher_t             publisher_ticks;
geometry_msgs__msg__Point32 ticks_msg;

volatile int  g_left_rpm  = 0;
volatile int  g_right_rpm = 0;
volatile bool g_new_cmd   = false;
unsigned long g_last_cmd_ms = 0;
const unsigned long CMD_WATCHDOG_MS = 1000;

enum McState { MC_IDLE, MC_WAIT_DISABLE, MC_WRITE_SPEED, MC_WAIT_SPEED, MC_WRITE_DIR };
McState       mc_state    = MC_IDLE;
unsigned long mc_timer    = 0;
int           mc_left     = 0;
int           mc_right    = 0;
int           cur_left    = 0;
int           cur_right   = 0;

const float   ENCODER_CPR  = 4000.0f;
float         g_left_ticks  = 0.0f;
float         g_right_ticks = 0.0f;
unsigned long last_odom_ms  = 0;

enum AgentState { WAITING_FOR_AGENT, AGENT_CONNECTED };
AgentState agentState = WAITING_FOR_AGENT;

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

void command_motors(int left, int right) {
    if (abs(left)  < RPM_DEADZONE) left  = 0;
    if (abs(right) < RPM_DEADZONE) right = 0;

    if (left == 0 && right == 0) {
        stop_motors_immediate();
        return;
    }

    mc_left  = left;
    mc_right = right;

    bool dir_change = (left  != 0 && cur_left  != 0 && ((left > 0) != (cur_left > 0)))  ||
                      (right != 0 && cur_right != 0 && ((right > 0) != (cur_right > 0)));

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

void process_motor_commands() {
    switch (mc_state) {
    case MC_IDLE:
        break;
    case MC_WAIT_DISABLE:
        if (millis() - mc_timer >= 20) mc_state = MC_WRITE_SPEED;
        break;
    case MC_WRITE_SPEED:
        if (mc_left  != 0) modbus_write(motorSerialL, LEFT_MOTOR_SLAVE_ID,  REG_SPEED, (uint16_t)abs(mc_left));
        if (mc_right != 0) modbus_write(motorSerialR, RIGHT_MOTOR_SLAVE_ID, REG_SPEED, (uint16_t)abs(mc_right));
        mc_timer = millis();
        mc_state = MC_WAIT_SPEED;
        break;
    case MC_WAIT_SPEED:
        if (millis() - mc_timer >= 20) mc_state = MC_WRITE_DIR;
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

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    motorSerialL.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX1_PIN, TX1_PIN);
    motorSerialR.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
    delay(100);

    stop_motors_immediate();
    delay(200);

    set_microros_transports();
}

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
        if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)) != RCL_RET_OK) {
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                stop_motors_immediate();
                destroy_entities();
                agentState = WAITING_FOR_AGENT;
                digitalWrite(LED_BUILTIN, LOW);
            }
            break;
        }

        if (g_new_cmd) {
            g_new_cmd     = false;
            g_last_cmd_ms = millis();
            command_motors(g_left_rpm, g_right_rpm);
        }

        process_motor_commands();

        if (g_last_cmd_ms > 0 && (millis() - g_last_cmd_ms) > CMD_WATCHDOG_MS) {
            stop_motors_immediate();
            g_last_cmd_ms = 0;
        }

        if (millis() - last_odom_ms >= 50) {
            unsigned long now = millis();
            float dt = (now - last_odom_ms) / 1000.0f;
            last_odom_ms = now;

            while (motorSerialL.available()) motorSerialL.read();
            while (motorSerialR.available()) motorSerialR.read();

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
```