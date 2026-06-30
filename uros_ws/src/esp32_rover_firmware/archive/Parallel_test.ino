#include <Arduino.h>
#include <RMCS2303drive.h> // Include the corrected library header

// --- User-defined GPIO pins for UART1 (Left Motor) ---
#define RX1_PIN_USER 16
#define TX1_PIN_USER 17
// --- User-defined GPIO pins for UART2 (Right Motor) ---
#define RX2_PIN_USER 14
#define TX2_PIN_USER 13

// --- Constants ---
#define LEFT_MOTOR_SLAVE_ID 7
#define RIGHT_MOTOR_SLAVE_ID 2
#define MODBUS_BAUD_RATE 9600
#define COMMAND_DELAY_MS 20 // ** NEW: Delay for driver processing **

// --- Hardware & Library Objects ---
HardwareSerial motorSerialL(1); // Use UART1 for Left Motor Driver
HardwareSerial motorSerialR(2); // Use UART2 for Right Motor Driver
RMCS2303 rmcsLeft;
RMCS2303 rmcsRight;


// --- MODIFIED FUNCTION ---
// Now includes small delays after each Modbus command for reliability.
void test_motor_with_library(RMCS2303 &controller, byte slave_id, int target_rpm, bool forward_motion, const char* motor_name) {
  Serial.print("Testing "); Serial.print(motor_name); Serial.print(" - RPM: "); Serial.print(target_rpm);
  
  int rmcs_motor_direction_cmd = forward_motion ? 0 : 1; 
  if (slave_id == 1) { // Assuming Slave ID 1 is the Left Motor
     rmcs_motor_direction_cmd = forward_motion ? 1 : 0; 
  }
  Serial.print(forward_motion ? " (Robot Fwd Intent -> " : " (Robot Rev Intent -> ");
  Serial.print("Motor Dir Cmd: "); Serial.print(rmcs_motor_direction_cmd); 
  Serial.println(rmcs_motor_direction_cmd == 0 ? "/CW)" : "/CCW)");

  // Send Speed command and wait briefly
  controller.Speed(slave_id, abs(target_rpm));
  delay(COMMAND_DELAY_MS); // ** CRITICAL DELAY **

  // Send Enable command and wait briefly
  controller.Enable_Digital_Mode(slave_id, rmcs_motor_direction_cmd);
  delay(COMMAND_DELAY_MS); // ** CRITICAL DELAY **
}

// --- MODIFIED FUNCTION ---
// Now includes a small delay after the Modbus command.
void stop_motor_with_library(RMCS2303 &controller, byte slave_id) {
  controller.Disable_Digital_Mode(slave_id, 0);
  delay(COMMAND_DELAY_MS); // ** CRITICAL DELAY **
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 RMCS-2303 Library Basic Test (PlatformIO)");

  motorSerialL.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX1_PIN_USER, TX1_PIN_USER);
  Serial.print("Left Motor UART (Serial1) on RX:"); Serial.print(RX1_PIN_USER); Serial.print(" TX:"); Serial.println(TX1_PIN_USER);
  
  motorSerialR.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX2_PIN_USER, TX2_PIN_USER);
  Serial.print("Right Motor UART (Serial2) on RX:"); Serial.print(RX2_PIN_USER); Serial.print(" TX:"); Serial.println(TX2_PIN_USER);

  rmcsLeft.begin(&motorSerialL, MODBUS_BAUD_RATE);
  Serial.println("RMCS Library initialized for Left Motor.");
  rmcsRight.begin(&motorSerialR, MODBUS_BAUD_RATE);
  Serial.println("RMCS Library initialized for Right Motor.");
  
  delay(200); 

  Serial.println("Ensuring motors are disabled initially...");
  stop_motor_with_library(rmcsLeft, LEFT_MOTOR_SLAVE_ID);
  stop_motor_with_library(rmcsRight, RIGHT_MOTOR_SLAVE_ID);
  delay(500); 

  Serial.println("Setup complete. Testing will begin in 3 seconds...");
  delay(3000);
}

void loop() {
  Serial.println("--- Test Sequence Start (with 0.5s delays) ---");

  // This sequence should now work reliably
  test_motor_with_library(rmcsLeft, LEFT_MOTOR_SLAVE_ID, 1000, true, "Left Motor");
  test_motor_with_library(rmcsRight, RIGHT_MOTOR_SLAVE_ID, 1000, true, "Right Motor");
  delay(500); // Motors run for 0.5 sec
  stop_motor_with_library(rmcsLeft, LEFT_MOTOR_SLAVE_ID);
  stop_motor_with_library(rmcsRight, RIGHT_MOTOR_SLAVE_ID);
  delay(500); // Motors are stopped for 0.5 sec

  test_motor_with_library(rmcsLeft, LEFT_MOTOR_SLAVE_ID, 800, false, "Left Motor");
  test_motor_with_library(rmcsRight, RIGHT_MOTOR_SLAVE_ID, 800, false, "Right Motor");
  delay(500); // Motors run for 0.5 sec

  stop_motor_with_library(rmcsLeft, LEFT_MOTOR_SLAVE_ID);   
  stop_motor_with_library(rmcsRight, RIGHT_MOTOR_SLAVE_ID);
  delay(500); // Motors are stopped for 0.5 sec
  
  Serial.println("--- Test Sequence End --- Pausing for 5 seconds before repeating ---");
  delay(500);
}