/**
 * motor_calibration.ino
 *
 * PURPOSE: Determine the correct physical spin direction of each motor
 * independently, so you can set LEFT_MOTOR_INVERTED and RIGHT_MOTOR_INVERTED
 * correctly in esp32_rover_firmware.ino.
 *
 * HOW TO USE:
 *   1. Flash this sketch to the ESP32 (rename esp32_rover_firmware.ino
 *      temporarily, or use a separate PlatformIO env).
 *   2. Open Serial Monitor at 115200 baud.
 *   3. Watch the motor spin and compare to what the Serial output says.
 *   4. Note your findings and set the flags in the main firmware.
 *
 * EXPECTED RESULTS for a correctly wired differential-drive rover:
 *   - "LEFT  motor → FORWARD (CW)" should make the left wheel push the
 *     rover forward (wheel spins so rover moves ahead).
 *   - "RIGHT motor → FORWARD (CW)" should make the right wheel push the
 *     rover forward.
 *   If either is opposite, set the corresponding INVERTED flag = true.
 *
 * NOTE: Only one motor runs at a time. The other is always stopped.
 * This sketch does NOT use micro-ROS — it runs standalone.
 */

#define RX1_PIN          16
#define TX1_PIN          17
#define RX2_PIN          14
#define TX2_PIN          13
#define MODBUS_BAUD_RATE 9600
#define LED_BUILTIN      2

#define LEFT_MOTOR_SLAVE_ID  7
#define RIGHT_MOTOR_SLAVE_ID 2

#define REG_CONTROL  2
#define REG_SPEED    14
#define CTRL_CW      257
#define CTRL_CCW     265
#define CTRL_DISABLE 256

#define TEST_RPM     80   // Safe, slow RPM for visual inspection
#define RUN_MS      2000  // How long each test phase runs (ms)
#define PAUSE_MS    1000  // Pause between tests

HardwareSerial motorSerialL(1);
HardwareSerial motorSerialR(2);

// ---------------------------------------------------------------------------
// Direct Modbus write — identical to main firmware
// ---------------------------------------------------------------------------
void modbus_write(HardwareSerial &port, byte slave_id,
                  int address, unsigned int data) {
    byte AddrHi = (address >> 8) & 0xFF;
    byte AddrLo =  address       & 0xFF;
    byte DataHi = (data   >> 8) & 0xFF;
    byte DataLo =  data         & 0xFF;
    const byte FC = 0x06;
    byte sum = slave_id + FC + AddrHi + AddrLo + DataHi + DataLo;
    byte lrc = (byte)((~sum) + 1);
    char frame[20];
    snprintf(frame, sizeof(frame), ":%02X%02X%02X%02X%02X%02X%02X\r\n",
             slave_id, FC, AddrHi, AddrLo, DataHi, DataLo, lrc);
    port.print(frame);
}

void stop_motor(HardwareSerial &port, byte slave_id) {
    modbus_write(port, slave_id, REG_CONTROL, CTRL_DISABLE);
}

void run_motor(HardwareSerial &port, byte slave_id, int rpm, bool cw) {
    modbus_write(port, slave_id, REG_SPEED, (uint16_t)abs(rpm));
    delay(20);
    modbus_write(port, slave_id, REG_CONTROL, cw ? CTRL_CW : CTRL_CCW);
}

// ---------------------------------------------------------------------------
// Setup — stop all motors on boot
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    motorSerialL.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX1_PIN, TX1_PIN);
    motorSerialR.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
    delay(200);

    stop_motor(motorSerialL, LEFT_MOTOR_SLAVE_ID);
    stop_motor(motorSerialR, RIGHT_MOTOR_SLAVE_ID);
    delay(500);

    Serial.println("====================================================");
    Serial.println(" MOTOR DIRECTION CALIBRATION");
    Serial.println("====================================================");
    Serial.println("Watch each motor carefully and note its spin direction.");
    Serial.println("Tests begin in 3 seconds...");
    Serial.println("====================================================");
    delay(3000);
    digitalWrite(LED_BUILTIN, HIGH);
}

// ---------------------------------------------------------------------------
// Loop — runs the calibration sequence once, then halts
// ---------------------------------------------------------------------------
bool calibration_done = false;

void loop() {
    if (calibration_done) {
        // Calibration complete — just blink LED slowly
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(500);
        return;
    }

    // ======================================================================
    // TEST 1: Left motor CW
    // ======================================================================
    Serial.println();
    Serial.println("TEST 1: LEFT motor → CW (CTRL=257)");
    Serial.println("  → Does the LEFT wheel push rover FORWARD? (Y/N)");
    run_motor(motorSerialL, LEFT_MOTOR_SLAVE_ID, TEST_RPM, true);
    delay(RUN_MS);
    stop_motor(motorSerialL, LEFT_MOTOR_SLAVE_ID);
    Serial.println("  Stopped.");
    delay(PAUSE_MS);

    // ======================================================================
    // TEST 2: Left motor CCW
    // ======================================================================
    Serial.println("TEST 2: LEFT motor → CCW (CTRL=265)");
    Serial.println("  → Does the LEFT wheel push rover FORWARD? (Y/N)");
    run_motor(motorSerialL, LEFT_MOTOR_SLAVE_ID, TEST_RPM, false);
    delay(RUN_MS);
    stop_motor(motorSerialL, LEFT_MOTOR_SLAVE_ID);
    Serial.println("  Stopped.");
    delay(PAUSE_MS);

    // ======================================================================
    // TEST 3: Right motor CW
    // ======================================================================
    Serial.println("TEST 3: RIGHT motor → CW (CTRL=257)");
    Serial.println("  → Does the RIGHT wheel push rover FORWARD? (Y/N)");
    run_motor(motorSerialR, RIGHT_MOTOR_SLAVE_ID, TEST_RPM, true);
    delay(RUN_MS);
    stop_motor(motorSerialR, RIGHT_MOTOR_SLAVE_ID);
    Serial.println("  Stopped.");
    delay(PAUSE_MS);

    // ======================================================================
    // TEST 4: Right motor CCW
    // ======================================================================
    Serial.println("TEST 4: RIGHT motor → CCW (CTRL=265)");
    Serial.println("  → Does the RIGHT wheel push rover FORWARD? (Y/N)");
    run_motor(motorSerialR, RIGHT_MOTOR_SLAVE_ID, TEST_RPM, false);
    delay(RUN_MS);
    stop_motor(motorSerialR, RIGHT_MOTOR_SLAVE_ID);
    Serial.println("  Stopped.");
    delay(PAUSE_MS);

    // ======================================================================
    // Summary instructions
    // ======================================================================
    Serial.println();
    Serial.println("====================================================");
    Serial.println(" CALIBRATION COMPLETE — Read and apply results:");
    Serial.println("====================================================");
    Serial.println(" In esp32_rover_firmware.ino, set:");
    Serial.println();
    Serial.println("   const bool LEFT_MOTOR_INVERTED  = ???;");
    Serial.println("   const bool RIGHT_MOTOR_INVERTED = ???;");
    Serial.println();
    Serial.println(" Rule: If Test 1 (CW) made left wheel go FORWARD:");
    Serial.println("   → LEFT_MOTOR_INVERTED = false");
    Serial.println(" If Test 2 (CCW) made left wheel go FORWARD:");
    Serial.println("   → LEFT_MOTOR_INVERTED = true");
    Serial.println();
    Serial.println(" Rule: If Test 3 (CW) made right wheel go FORWARD:");
    Serial.println("   → RIGHT_MOTOR_INVERTED = false");
    Serial.println(" If Test 4 (CCW) made right wheel go FORWARD:");
    Serial.println("   → RIGHT_MOTOR_INVERTED = true");
    Serial.println("====================================================");

    calibration_done = true;
}
