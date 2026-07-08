# Rhino RMCS-2303 DC Servo Drive Operating Manual

This document is the operating manual for the **Rhino Motion Controls DC Servo Drive (Model: RMCS-2303)**. It contains technical specifications, pinout descriptions, hardware jumper slave ID configurations, Modbus register maps, and step-by-step operating mode settings.

---

## 1. Introduction & Salient Features

The RMCS-2303 is a high-performance closed-loop DC servo drive designed for operating DC servo motors with encoder feedback within a voltage range of **10V to 30V DC**.

* **Closed-loop control:** Provides closed-loop speed and position control for DC servo motor systems.
* **Speed regulation:** Motor speeds are maintained at programmed values irrespective of changes in supply voltage.
* **Constant torque:** Rated motor torque is available across the entire speed range.
* **Control modes:** Supports three control modes:
  * **Mode 0:** Analog Speed Control Mode
  * **Mode 1:** Digital Speed Control Mode
  * **Mode 2:** Position Control Mode
* **Protection mechanisms:** Features short-circuit protection for motor outputs, over-voltage/under-voltage protection, and can survive accidental motor disconnects while powered up.
* **Communication interface:** Configured using the **Modbus RTU ASCII** protocol via a 3.3V-TTL UART interface.
* **Addressing options:** Device address (Slave ID) can be set from 1 to 7 using physical jumpers (hardware setting) or from 1 to 247 using software configuration.

---

## 2. Technical Specifications & Electrical Ratings

| Specification | Min | Max | Units | Comments |
| :--- | :---: | :---: | :---: | :--- |
| **Supply Voltage** | 10 | 30 | Volts DC | Connected between +Ve (VDD) and GND |
| **Phase Current** | 0.5 | 3 | Amps | Peak rating of 5 Amps per phase |

---

## 3. Physical Pinout Descriptions

### 3.1. Drive Configuration and UART Control (Pins 1–7)

Pins 1 through 7 are used for configuration, analog speed input, and UART control logic:

| Pin No. | Label | Type | Description |
| :---: | :---: | :---: | :--- |
| **1** | **GND** | Power | Common Logic Ground |
| **2** | **RXD** | Input | UART Serial Receive Pin (3.3V-TTL) |
| **3** | **TXD** | Output | UART Serial Transmit Pin (3.3V-TTL) |
| **4** | **ENA** | Input | Enable Pin (Connect to GND to enable motor in Analog Mode) |
| **5** | **ADC** | Input | Analog control input (0V to 3.3V max - do not exceed 3.3V) |
| **6** | **DIR** | Input | Direction Control Pin (Connect to GND to change direction in Analog Mode) |
| **7** | **BRK** | Input | Brake Pin (Connect to GND to lock motor) |

### 3.2. Motor and Power Supply Connections (Pins 8–17)

Pins 8 through 17 connect the drive to the DC motor, its encoder, and the main DC power source:

| Pin No. | Label | Color (Typ.) | Description |
| :---: | :---: | :---: | :--- |
| **8** | **GND** | Black | Encoder Ground |
| **9** | **+5VDC** | Brown | +5V output power for encoder logic |
| **10** | **ENC_B** | Red | Encoder Channel B input / Hall W |
| **11** | **ENC_A** | Orange | Encoder Channel A input / Hall U |
| **12** | **Hall U** | — | Hall U sensor input (for BLDC motors) |
| **13** | **W** | — | Motor Phase W connection (for BLDC motors) |
| **14** | **Motor- / V** | Yellow | Motor negative terminal / Phase V |
| **15** | **Motor+ / U** | Green | Motor positive terminal / Phase U |
| **16** | **VDD** | Red (Thick) | Main Power Supply input (10V to 30V DC) |
| **17** | **GND** | Black (Thick)| Main Power Supply Ground |

---

## 4. Slave ID Address Configuration (Jumpers JP1–JP3)

Hardware addressing is defined by physical jumpers JP1, JP2, and JP3. A state of `0` indicates the jumper is OPEN; a state of `1` indicates it is CLOSED (connected).

> [!NOTE]
> The default mode if no jumpers are connected (all 0) is Slave ID 7.

| Slave ID | Jumper JP1 (Bit 0) | Jumper JP2 (Bit 1) | Jumper JP3 (Bit 2) |
| :---: | :---: | :---: | :---: |
| **1** | 0 | 0 | 1 |
| **2** | 0 | 1 | 0 |
| **3** | 0 | 1 | 1 |
| **4** | 1 | 0 | 0 |
| **5** | 1 | 0 | 1 |
| **6** | 1 | 1 | 0 |
| **7** | 1 | 1 | 1 |

---

## 5. Mode Configuration Guide

Configure the drive registers using Modbus RTU ASCII serial commands (9600 Baud, 8N1, No Parity). All commands shown are written to the drive using **Modbus Function 06 (Write Single Register)**.

### 5.1. Mode 0: Analog Speed Control Mode
In this mode, motor speed is regulated using an external potentiometer connected to the ADC pin (Pin 5) through a voltage divider.

1. **Hardware connections:**
   * Wire the potentiometer to the ADC pin through a voltage divider to limit input voltage to $3.3\text{ V}$.
   * Connect ENA (Pin 4) to GND (Pin 8) to enable the motor.
2. **Configuration commands:**
   * Write Hex value `0000` to Address 2 to select Mode 0.
   * Write Hex value `0001` to Address 2 to enable the motor.
3. **Controls:**
   * Adjust potentiometer to change speed.
   * Ground DIR (Pin 6) to change direction.
   * Ground BRK (Pin 7) to brake/lock the motor.

### 5.2. Mode 1: Digital Speed Control Mode
In this mode, motor speed and direction are controlled by sending Modbus write commands.

1. **Configuration commands:**
   * Write Hex value `0100` to Address 2 to select Mode 1 (digital mode).
   * Write Hex value `0101` to Address 2 to enable the motor.
2. **Speed command:**
   * Write target speed RPM (Decimal 0 to 18000) directly to Address 14. E.g., send Hex `1000` (4096 RPM base motor speed).
3. **Direction change:**
   * Write Hex `0109` to Address 2 to invert the motor direction.
4. **Stop command:**
   * Write Hex `0000` to Address 2 to stop the motor.

### 5.3. Mode 2: Position Control Mode
In this mode, the motor runs to a target encoder position. The command position is input as a 32-bit value split across two 16-bit registers (Address 16 LSB and Address 18 MSB).

1. **Lines per rotation setup:**
   * Write the lines-per-rotation count of your encoder to Address 10.
     * *High Precision Encoder:* 334 lines (Hex `14E`). Since it is a quadrature encoder, one encoder turn is $334 \times 4 = 1336$ steps.
     * *Standard Quad Encoder:* 41 lines. One encoder turn is $41 \times 4 = 164$ steps.
2. **Configuration commands:**
   * Write Hex value `0200` to Address 2 to select Mode 2 (position control).
   * Write initial target values:
     * Write LSB position value to Address 16.
     * Write MSB position value to Address 18.
   * Write Hex value `0201` to Address 2 to enable the motor and execute the position move.
3. **Target position calculations (Example):**
   * To rotate a geared motor with a 90:1 gear ratio and a High Precision Encoder (1336 steps/rotation) by exactly 1 full turn of the output shaft:
     $$\text{Target Steps} = 1336 \times 90 = 120,240 \text{ steps} \implies \text{Hex: } \texttt{0x1D5B0}$$
     * Write the LSB part (`D5B0`) to Address 16.
     * Write the MSB part (`1`) to Address 18.
     * Write Hex `0201` to Address 2 to execute the movement.

### 5.4. Saving and Resetting Settings
* **Save settings to EEPROM:** Write Hex value `07FF` to Address 0. Settings will persist across power cycles.
* **Reset settings to factory defaults:** Write Hex value `0` to Address 4, then save by writing `07FF` to Address 0. Power cycle the drive to load default values.

---

## 6. Appendix: Modbus Configuration Registers

| Control Register | Modbus Input Register | No. of Bits | Max Value | Default (Hex) | Description |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **DEVICE_MODBUS_ADDRESS** | 1 | - | — | `0007` | Device Modbus Slave Address |
| **INP_CONTROL_BYTE** | 2 | 8 | 255 | `0000` | Input Control Byte (Mode + enable/direction/brake) |
| **INP_MODE_BYTE** | 3 | 8 | 255 | `2000` | Input Mode Byte (read-only state feedback) |
| **VP_GAIN_BYTE** | 4 | 8 | 255 | `FF20` | Velocity Proportional gain (P gain) |
| **VI_GAIN_BYTE** | 6 | 8 | 255 | `FF10` | Velocity Integral gain (I gain) |
| **VF_GAIN_BYTE** | 8 | 8 | 255 | `FF20` | Velocity Feed Forward gain (F gain) |
| **LINES_PER_ROT** | 10 | 16 | 65535 | `014E` | Encoder Lines per rotation |
| **TRP_ACL_WORD** | 12 | 16 | 65535 | `4E20` | Motor acceleration rate |
| **TRP_SPD_WORD** | 14 | 16 | 65535 | `0800` | Target Speed (used in Mode 1) |
| **CMD_LSB_WORD** | 16 | 16 | 65535 | `0000` | Target Position LSB (used in Mode 2) |
| **CMD_MSB_WORD** | 18 | 16 | 65535 | `0000` | Target Position MSB (used in Mode 2) |
| **POS_LSB_WORD** | 20 | 16 | 65535 | `0000` | Current Position LSB (read-only feedback) |
| **POS_MSB_WORD** | 22 | 16 | 65535 | `0000` | Current Position MSB (read-only feedback) |
| **ACT_SPD_WORD** | 24 | 16 | 65535 | `0048` | Current Speed (read-only feedback) |

---

## 7. Troubleshooting

* **Timeout Error:** Occurs when the driver does not respond. Check that the Modbus Slave ID matches your jumpers and connection configuration.
* **Byte Missing/Corrupt Error:** Check the serial wiring connection quality, ensure logic ground (GND) is shared, and power cycle the drive. Verify JP1–JP3 settings.
* **Position Control Errors:** Check that `LINES_PER_ROT` is set correctly. Ensure acceleration and speed parameters are set within motor limits. Swap direction command only when speed is low or stopped.
