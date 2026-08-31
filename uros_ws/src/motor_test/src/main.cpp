/**
 * motor_test v4 — Three-phase isolated motor validation
 *
 * Build with one of three environments:
 *   TEST_PHASE=1  left_only   — Left motor only (UART1/ID=7)
 *   TEST_PHASE=2  right_only  — Right motor only (UART2/ID=2)
 *   TEST_PHASE=3  both        — Both motors, WASD-style sequences
 *
 * Physical direction (confirmed by v3 ACK test):
 *   LEFT  motor: CW = forward,  CCW = backward
 *   RIGHT motor: CW = BACKWARD, CCW = FORWARD  ← physically inverted
 *
 * So in TEST_PHASE=3 the firmware applies the same inversion as firmware_v2:
 *   right motor forward → send CCW
 *   right motor backward → send CW
 */

#include <Arduino.h>

#define RX1_PIN 16
#define TX1_PIN 17
#define RX2_PIN 14
#define TX2_PIN 13
#define BAUD    9600
#define LED     2

#define LEFT_ID      7
#define RIGHT_ID     2
#define REG_SPEED   14
#define REG_CTRL     2
#define CTRL_CW     257    // 0x0101 — Mode 1, Enable, CW
#define CTRL_CCW    265    // 0x0109 — Mode 1, Enable, CCW
#define CTRL_STOP     0    // 0x0000 — actual stop (0x0100 is only "Mode 1 select", NOT stop)

#define TEST_RPM    50     // RPM for all test sequences
#define HOLD_MS   3000     // How long each direction is held

HardwareSerial portL(1);   // UART1 — left  motor driver
HardwareSerial portR(2);   // UART2 — right motor driver

// =============================================================================
// Modbus helper — send, flush, wait 60ms for ACK, drain, log ACK byte count
// =============================================================================
int mbus(HardwareSerial &port, byte slave, int reg, unsigned int data, const char *label) {
    byte sum = slave + 0x06 +
               ((reg  >> 8) & 0xFF) + (reg  & 0xFF) +
               ((data >> 8) & 0xFF) + (data & 0xFF);
    byte lrc = (byte)((~sum) + 1);
    char frame[20];
    snprintf(frame, sizeof(frame), ":%02X06%04X%04X%02X\r\n", slave, reg, data, lrc);
    port.print(frame);
    port.flush();
    delay(60);
    int ack = 0;
    while (port.available()) { port.read(); ack++; }
    Serial.printf("  [%s] %s  ACK=%d\n", label, frame, ack);
    return ack;
}

// Zero the speed target first (PID ramps to rest), then send the real stop.
void disable_left()  {
    mbus(portL, LEFT_ID,  REG_SPEED, 0,        "LEFT-ZERO");
    mbus(portL, LEFT_ID,  REG_CTRL, CTRL_STOP, "LEFT-STOP");
}
void disable_right() {
    mbus(portR, RIGHT_ID, REG_SPEED, 0,        "RIGHT-ZERO");
    mbus(portR, RIGHT_ID, REG_CTRL, CTRL_STOP, "RIGHT-STOP");
}
void disable_all()   { disable_left(); disable_right(); }

void setup() {
    Serial.begin(115200);
    pinMode(LED, OUTPUT);
    portL.begin(BAUD, SERIAL_8N1, RX1_PIN, TX1_PIN);
    portR.begin(BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
    delay(500);

#if TEST_PHASE == 1
    Serial.println("\n=== MOTOR TEST v4 — Phase 1: LEFT MOTOR ONLY ===");
    Serial.println("Expected behavior:");
    Serial.println("  CW  → LEFT wheel rolls FORWARD");
    Serial.println("  CCW → LEFT wheel rolls BACKWARD");
    Serial.println("Right motor is disabled throughout.\n");
#elif TEST_PHASE == 2
    Serial.println("\n=== MOTOR TEST v4 — Phase 2: RIGHT MOTOR ONLY ===");
    Serial.println("Expected behavior (right motor is physically inverted):");
    Serial.println("  CW  → RIGHT wheel rolls BACKWARD  (physically inverted)");
    Serial.println("  CCW → RIGHT wheel rolls FORWARD   (physically inverted)");
    Serial.println("Left motor is disabled throughout.\n");
#elif TEST_PHASE == 3
    Serial.println("\n=== MOTOR TEST v4 — Phase 3: BOTH MOTORS — WASD sequences ===");
    Serial.println("Sequences with correct inversion applied to right motor:");
    Serial.println("  W Forward : L=CW,  R=CCW → both wheels roll forward");
    Serial.println("  S Backward: L=CCW, R=CW  → both wheels roll backward");
    Serial.println("  A Left    : L=CCW, R=CCW → left back, right fwd = left turn");
    Serial.println("  D Right   : L=CW,  R=CW  → left fwd, right back = right turn\n");
#endif

    disable_all();
    delay(1000);
}

// =============================================================================
// Phase 1 — Left motor only
// =============================================================================
#if TEST_PHASE == 1

void loop() {
    Serial.println("--- LEFT CW (expect FORWARD) ---");
    mbus(portL, LEFT_ID, REG_SPEED, TEST_RPM, "LEFT-SPD");
    mbus(portL, LEFT_ID, REG_CTRL,  CTRL_CW,  "LEFT-CW");
    delay(HOLD_MS);
    disable_left();
    delay(1000);

    Serial.println("--- LEFT CCW (expect BACKWARD) ---");
    mbus(portL, LEFT_ID, REG_SPEED, TEST_RPM, "LEFT-SPD");
    mbus(portL, LEFT_ID, REG_CTRL,  CTRL_CCW, "LEFT-CCW");
    delay(HOLD_MS);
    disable_left();
    delay(1000);

    Serial.println("=== cycle done ===\n");
}

// =============================================================================
// Phase 2 — Right motor only (with physical inversion noted)
// =============================================================================
#elif TEST_PHASE == 2

void loop() {
    // CW → physically backward for right motor
    Serial.println("--- RIGHT CW (expect BACKWARD — physically inverted) ---");
    mbus(portR, RIGHT_ID, REG_SPEED, TEST_RPM, "RIGHT-SPD");
    mbus(portR, RIGHT_ID, REG_CTRL,  CTRL_CW,  "RIGHT-CW");
    delay(HOLD_MS);
    disable_right();
    delay(1000);

    // CCW → physically forward for right motor
    Serial.println("--- RIGHT CCW (expect FORWARD — physically inverted) ---");
    mbus(portR, RIGHT_ID, REG_SPEED, TEST_RPM, "RIGHT-SPD");
    mbus(portR, RIGHT_ID, REG_CTRL,  CTRL_CCW, "RIGHT-CCW");
    delay(HOLD_MS);
    disable_right();
    delay(1000);

    Serial.println("=== cycle done ===\n");
}

// =============================================================================
// Phase 3 — Both motors, WASD sequences with correct inversion
// =============================================================================
#elif TEST_PHASE == 3

// Set both motors to speed, then direction, then hold, then stop.
// right_dir is the RAW Modbus command for right motor (inversion already applied by caller).
void run_both(unsigned int left_dir, unsigned int right_dir, unsigned long hold_ms) {
    // Speed
    mbus(portL, LEFT_ID,  REG_SPEED, TEST_RPM, "L-SPD");
    mbus(portR, RIGHT_ID, REG_SPEED, TEST_RPM, "R-SPD");
    // Direction
    mbus(portL, LEFT_ID,  REG_CTRL, left_dir,  "L-DIR");
    mbus(portR, RIGHT_ID, REG_CTRL, right_dir, "R-DIR");
    delay(hold_ms);
    // Ramp speed to zero before disabling so the driver decelerates cleanly.
    // Then wait 2000ms for the motor to physically coast to a full stop before
    // the next direction command arrives.
    mbus(portL, LEFT_ID,  REG_SPEED, 0, "L-ZERO");
    mbus(portR, RIGHT_ID, REG_SPEED, 0, "R-ZERO");
    disable_all();
    delay(2000);
}

void loop() {
    // W — Forward: Left CW (fwd), Right CCW (fwd, inverted)
    Serial.println("--- W FORWARD: L=CW, R=CCW ---");
    run_both(CTRL_CW, CTRL_CCW, HOLD_MS);

    // S — Backward: Left CCW (bwd), Right CW (bwd, inverted)
    Serial.println("--- S BACKWARD: L=CCW, R=CW ---");
    run_both(CTRL_CCW, CTRL_CW, HOLD_MS);

    // A — Turn Left: Left CCW (bwd), Right CCW (fwd, inverted)
    Serial.println("--- A TURN LEFT: L=CCW, R=CCW ---");
    run_both(CTRL_CCW, CTRL_CCW, HOLD_MS);

    // D — Turn Right: Left CW (fwd), Right CW (bwd, inverted)
    Serial.println("--- D TURN RIGHT: L=CW, R=CW ---");
    run_both(CTRL_CW, CTRL_CW, HOLD_MS);

    Serial.println("=== cycle done ===\n");
}

// =============================================================================
// Phase 4 — Register dump + live drive diagnostic (right motor focus)
// =============================================================================
#elif TEST_PHASE == 4

// Modbus function 03 — read one register, parse and LRC-check the response.
bool mread(HardwareSerial &port, byte slave, int reg, uint16_t &val) {
    while (port.available()) port.read();
    byte sum = slave + 0x03 + ((reg >> 8) & 0xFF) + (reg & 0xFF) + 0x00 + 0x01;
    byte lrc = (byte)((~sum) + 1);
    char f[20];
    snprintf(f, sizeof(f), ":%02X03%04X0001%02X\r\n", slave, reg, lrc);
    port.print(f);
    port.flush();
    unsigned long dl = millis() + 150;
    char buf[24]; int n = 0;
    while (millis() < dl) {
        while (port.available()) {
            char c = port.read();
            if (n < 23) buf[n++] = c;
            if (c == '\n') {
                buf[n] = 0;
                if (n < 13 || buf[0] != ':') return false;
                auto h2 = [&](int i) -> byte {
                    char h[3] = {buf[i], buf[i+1], 0};
                    return (byte)strtol(h, NULL, 16);
                };
                if (h2(3) != 3 || h2(5) != 2) return false;
                val = ((uint16_t)h2(7) << 8) | h2(9);
                return true;
            }
        }
    }
    return false;
}

void dump_regs(const char *name, HardwareSerial &port, byte slave) {
    const int   regs[]  = {1, 2, 3, 4, 6, 8, 10, 12, 14, 20, 22, 24};
    const char *names[] = {"ADDR", "CTRL", "MODE", "P-GAIN", "I-GAIN", "F-GAIN",
                           "LINES", "ACCEL", "SPEED", "POS_L", "POS_M", "ACT_SPD"};
    Serial.printf("\n--- %s (ID=%d) register dump ---\n", name, slave);
    for (int i = 0; i < 12; i++) {
        uint16_t v;
        if (mread(port, slave, regs[i], v))
            Serial.printf("  reg %2d %-8s = %5u (0x%04X)\n", regs[i], names[i], v, v);
        else
            Serial.printf("  reg %2d %-8s = READ FAILED\n", regs[i], names[i]);
        delay(20);
    }
}

void loop() {
    dump_regs("LEFT",  portL, LEFT_ID);
    dump_regs("RIGHT", portR, RIGHT_ID);

    Serial.println("\n--- Driving RIGHT CCW 50 RPM for ~4s, sampling live ---");
    mbus(portR, RIGHT_ID, REG_SPEED, TEST_RPM, "R-SPD");
    mbus(portR, RIGHT_ID, REG_CTRL,  CTRL_CCW, "R-CCW");
    for (int i = 0; i < 10; i++) {
        uint16_t s = 0, p = 0, m = 0;
        bool oks = mread(portR, RIGHT_ID, 24, s);
        bool okp = mread(portR, RIGHT_ID, 20, p);
        bool okm = mread(portR, RIGHT_ID, 3,  m);
        Serial.printf("  t=%4dms  ACT_SPD=%s%-5u  POS_LSB=%s%-5u  MODE=%s0x%04X\n",
                      i * 400,
                      oks ? "" : "FAIL ", s,
                      okp ? "" : "FAIL ", p,
                      okm ? "" : "FAIL ", m);
        delay(300);
    }
    disable_right();

    Serial.println("\n=== diag cycle done — repeating in 8s ===\n");
    delay(8000);
}

// =============================================================================
// Phase 5 — Direction-semantics probe
// Determines whether 0x0101/0x0109 are ABSOLUTE direction commands or whether
// 0x0109 TOGGLES a latched direction (manual says "invert the motor direction").
// Runs a fixed control-word sequence on each motor, logging the encoder
// direction sign for every step.
// =============================================================================
#elif TEST_PHASE == 5

bool mread(HardwareSerial &port, byte slave, int reg, uint16_t &val) {
    while (port.available()) port.read();
    byte sum = slave + 0x03 + ((reg >> 8) & 0xFF) + (reg & 0xFF) + 0x00 + 0x01;
    byte lrc = (byte)((~sum) + 1);
    char f[20];
    snprintf(f, sizeof(f), ":%02X03%04X0001%02X\r\n", slave, reg, lrc);
    port.print(f);
    port.flush();
    unsigned long dl = millis() + 150;
    char buf[24]; int n = 0;
    while (millis() < dl) {
        while (port.available()) {
            char c = port.read();
            if (n < 23) buf[n++] = c;
            if (c == '\n') {
                buf[n] = 0;
                if (n < 13 || buf[0] != ':') return false;
                auto h2 = [&](int i) -> byte {
                    char h[3] = {buf[i], buf[i+1], 0};
                    return (byte)strtol(h, NULL, 16);
                };
                if (h2(3) != 3 || h2(5) != 2) return false;
                val = ((uint16_t)h2(7) << 8) | h2(9);
                return true;
            }
        }
    }
    return false;
}

// Read full 32-bit position (motor stopped when called → no LSB/MSB tearing).
long read_pos(HardwareSerial &port, byte slave) {
    uint16_t lsb = 0, msb = 0;
    if (!mread(port, slave, 20, lsb)) return 0x7FFFFFFF;
    delay(20);
    if (!mread(port, slave, 22, msb)) return 0x7FFFFFFF;
    return (long)(((uint32_t)msb << 16) | lsb);
}

void probe_motor(const char *name, HardwareSerial &port, byte slave) {
    // Control-word sequence. 1=0x0101, 9=0x0109. Stop (0x0000) between steps.
    const unsigned int seq[] = {0x0101, 0x0101, 0x0109, 0x0109, 0x0101, 0x0109, 0x0101};
    Serial.printf("\n=== %s (ID=%d) direction-semantics probe ===\n", name, slave);
    Serial.println("cmd     enc_delta   sign");

    // Ensure stopped baseline
    mbus(port, slave, REG_SPEED, 0, "zero");
    mbus(port, slave, REG_CTRL, CTRL_STOP, "stop");
    delay(1500);

    for (int i = 0; i < 7; i++) {
        long p0 = read_pos(port, slave);
        mbus(port, slave, REG_SPEED, TEST_RPM, "spd");
        mbus(port, slave, REG_CTRL, seq[i], "ctl");
        delay(2000);                                   // run 2s
        mbus(port, slave, REG_SPEED, 0, "zero");
        mbus(port, slave, REG_CTRL, CTRL_STOP, "stop");
        delay(1500);                                   // coast to full rest
        long p1 = read_pos(port, slave);
        long d  = p1 - p0;
        Serial.printf("0x%04X  %9ld    %s\n", seq[i], d,
                      (d > 1000) ? "+" : (d < -1000) ? "-" : "0 (NO MOTION)");
    }
}

void loop() {
    probe_motor("LEFT",  portL, LEFT_ID);
    probe_motor("RIGHT", portR, RIGHT_ID);
    Serial.println("\n=== probe done — repeating in 10s ===\n");
    delay(10000);
}

// =============================================================================
// Phase 6 — Motor health report card
// Runs each motor through repeated forward/reverse drives from a full stop and
// grades: (1) does it move, (2) is speed regulation repeatable, (3) is
// direction deterministic, (4) do the two commands give OPPOSITE directions.
// Prints an explicit PASS/FAIL verdict per motor.
// =============================================================================
#elif TEST_PHASE == 6

bool mread(HardwareSerial &port, byte slave, int reg, uint16_t &val) {
    while (port.available()) port.read();
    byte sum = slave + 0x03 + ((reg >> 8) & 0xFF) + (reg & 0xFF) + 0x00 + 0x01;
    byte lrc = (byte)((~sum) + 1);
    char f[20];
    snprintf(f, sizeof(f), ":%02X03%04X0001%02X\r\n", slave, reg, lrc);
    port.print(f);
    port.flush();
    unsigned long dl = millis() + 150;
    char buf[24]; int n = 0;
    while (millis() < dl) {
        while (port.available()) {
            char c = port.read();
            if (n < 23) buf[n++] = c;
            if (c == '\n') {
                buf[n] = 0;
                if (n < 13 || buf[0] != ':') return false;
                auto h2 = [&](int i) -> byte {
                    char h[3] = {buf[i], buf[i+1], 0};
                    return (byte)strtol(h, NULL, 16);
                };
                if (h2(3) != 3 || h2(5) != 2) return false;
                val = ((uint16_t)h2(7) << 8) | h2(9);
                return true;
            }
        }
    }
    return false;
}

// Full 32-bit position — read only when the motor is stopped (no tearing).
long read_pos(HardwareSerial &port, byte slave) {
    uint16_t lsb = 0, msb = 0;
    if (!mread(port, slave, 20, lsb)) return 0x7FFFFFFF;
    delay(20);
    if (!mread(port, slave, 22, msb)) return 0x7FFFFFFF;
    return (long)(((uint32_t)msb << 16) | lsb);
}

// One stopped-to-stopped run at TEST_RPM in the given raw direction.
// Returns net encoder displacement (0x7FFFFFFF sentinel if a read failed).
long run_once(HardwareSerial &port, byte slave, unsigned int dir) {
    long p0 = read_pos(port, slave);
    if (p0 == 0x7FFFFFFF) return 0x7FFFFFFF;
    mbus(port, slave, REG_SPEED, TEST_RPM, "spd");
    mbus(port, slave, REG_CTRL,  dir,      "run");
    delay(2000);
    mbus(port, slave, REG_SPEED, 0,        "zero");
    mbus(port, slave, REG_CTRL,  CTRL_STOP,"stop");
    delay(1500);
    long p1 = read_pos(port, slave);
    if (p1 == 0x7FFFFFFF) return 0x7FFFFFFF;
    return p1 - p0;
}

void health_check(const char *name, HardwareSerial &port, byte slave) {
    // At 50 RPM for 2s with a 334-line (1336 count/rev) encoder we expect
    // ~2230 counts. 500 gives clean separation from noise/no-motion.
    const long MOVE_THRESH = 500;   // counts — below this = "no motion"
    Serial.printf("\n========================================\n");
    Serial.printf("  MOTOR HEALTH REPORT — %s (ID=%d)\n", name, slave);
    Serial.printf("========================================\n");

    // --- Confirm the drive is reachable and read its config ---
    uint16_t lines = 0, pgain = 0;
    bool comms = mread(port, slave, 10, lines) && mread(port, slave, 4, pgain);
    if (!comms) {
        Serial.printf("  COMMS: NO RESPONSE — driver unpowered or unwired.\n");
        Serial.printf("  VERDICT: CANNOT TEST\n");
        return;
    }
    Serial.printf("  Comms OK | LINES/rot=%u  P-gain=%u\n", lines, pgain);

    // baseline stop
    mbus(port, slave, REG_SPEED, 0, "zero");
    mbus(port, slave, REG_CTRL, CTRL_STOP, "stop");
    delay(1500);

    // --- 3 runs each direction ---
    long cw[3], ccw[3];
    Serial.printf("  Running 3x CW (0x0101) and 3x CCW (0x0109)...\n");
    for (int i = 0; i < 3; i++) cw[i]  = run_once(port, slave, CTRL_CW);
    for (int i = 0; i < 3; i++) ccw[i] = run_once(port, slave, CTRL_CCW);

    Serial.printf("  CW  deltas: %9ld %9ld %9ld\n", cw[0], cw[1], cw[2]);
    Serial.printf("  CCW deltas: %9ld %9ld %9ld\n", ccw[0], ccw[1], ccw[2]);

    // --- Grade ---
    auto sign = [](long v) -> int { return (v > 500) ? 1 : (v < -500) ? -1 : 0; };
    bool moved = true;
    for (int i = 0; i < 3; i++) {
        if (labs(cw[i]) < MOVE_THRESH || labs(ccw[i]) < MOVE_THRESH) moved = false;
    }
    bool cw_consistent  = (sign(cw[0])  != 0 && sign(cw[0])  == sign(cw[1])  && sign(cw[1])  == sign(cw[2]));
    bool ccw_consistent = (sign(ccw[0]) != 0 && sign(ccw[0]) == sign(ccw[1]) && sign(ccw[1]) == sign(ccw[2]));
    bool opposite = (cw_consistent && ccw_consistent && sign(cw[0]) != sign(ccw[0]));

    // speed regulation: CW magnitudes within 30% of their mean
    long m = (labs(cw[0]) + labs(cw[1]) + labs(cw[2])) / 3;
    bool speed_ok = m > 0;
    for (int i = 0; i < 3; i++) if (labs(labs(cw[i]) - m) > (m * 3 / 10)) speed_ok = false;

    Serial.printf("  ----------------------------------------\n");
    Serial.printf("  [%s] Motor spins under command\n",        moved ? "PASS" : "FAIL");
    Serial.printf("  [%s] Speed regulation repeatable (±30%%)\n", speed_ok ? "PASS" : "FAIL");
    Serial.printf("  [%s] CW  direction deterministic\n",      cw_consistent  ? "PASS" : "FAIL");
    Serial.printf("  [%s] CCW direction deterministic\n",      ccw_consistent ? "PASS" : "FAIL");
    Serial.printf("  [%s] CW and CCW are OPPOSITE\n",          opposite ? "PASS" : "FAIL");
    Serial.printf("  ----------------------------------------\n");
    if (moved && speed_ok && opposite)
        Serial.printf("  VERDICT: MOTOR FUNCTIONAL — direction reliable\n");
    else if (moved && speed_ok && !opposite)
        Serial.printf("  VERDICT: SPINS + REGULATES, but DIRECTION UNRELIABLE\n"
                      "           (closed-loop cannot resolve direction —\n"
                      "            encoder feedback or drive logic fault)\n");
    else if (!moved)
        Serial.printf("  VERDICT: MOTOR DOES NOT SPIN RELIABLY (power/wiring)\n");
    else
        Serial.printf("  VERDICT: FAULTY — see failed criteria above\n");
}

void loop() {
    health_check("LEFT",  portL, LEFT_ID);
    health_check("RIGHT", portR, RIGHT_ID);
    Serial.printf("\n=== full health cycle done — repeating in 12s ===\n\n");
    delay(12000);
}

#else
#error "Define TEST_PHASE=1..6 via build_flags"
#endif
