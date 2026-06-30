# Context: RPLIDAR A1M8 360-Degree Laser Scanner Integration

This document serves as the technical context and specification reference for integrating the **Slamtec RPLIDAR A1 (Model: A1M8)** into the antigravity rover project. The following data is compiled directly from the official manufacturer datasheet (Revision 1.0, 2016-07-04).

---

## 1. System Overview & Mechanism

The RPLIDAR A1 is a low-cost, 360-degree, 2D laser scanner (LIDAR) solution. It utilizes a **laser triangulation measurement system**, enabling precise mapping, localization, and object/environment modeling.

* **Rotation Direction:** Clockwise scanning.
* **Operating Principle:** Emits a modulated infrared laser signal reflected by ambient objects. The returning signal is captured by a vision acquisition system, and processed by an embedded Digital Signal Processor (DSP) to calculate real-time distance and heading values.
* **Environmental Compatibility:** Works optimally in all indoor environments and outdoor environments *without* direct sunlight (modulated laser prevents interference from standard ambient light).

---

## 2. Measurement & Optical Performance

The core specifications for the **A1M8** model must be hardcoded into the rover's navigation and slam configuration files:

| Parameter | Unit | Value / Specification |
| :--- | :--- | :--- |
| **Distance Range** | Meter (m) | `0.15 - 6` (Tested on white objects) |
| **Angular Range** | Degree | `0 - 360` |
| **Distance Resolution** | mm | `< 0.5` (at ranges `< 1.5 meters`) / `< 1%` of total distance across full range |
| **Angular Resolution** | Degree | `≤ 1.0` |
| **Sample Duration** | ms | `0.5` |
| **Sample Frequency** | Hz | `≥ 2000` (Typical) / Max: `2010` |
| **Scan Rate (Frequency)** | Hz | `5.5` (Typical, sampling 360 points/round) / Configurable up to `10` max |
| **Laser Wavelength** | nm | `785` (Typical) / Range: `775 - 795` (Infrared band) |
| **Laser Peak Power** | mW | `3` (Typical) / `5` (Max) |
| **Pulse Length** | µs | `110` (Typical) / `300` (Max) |
| **Laser Safety Class** | N/A | **Class I Standard** (Modulated pulses ensure human/pet eye safety) |

---

## 3. Communication Interface & Protocol

The system interacts via a standard serial interface. Custom hardware bridges (like USB) can be used, but the raw core relies on standard logic levels.

### Serial Configuration
* **Baud Rate:** `115200 bps`
* **Working Mode:** `8N1` (8 data bits, No parity, 1 stop bit)

### Logic Levels (3.3V-TTL UART)
* **Output High Voltage:** `2.9V - 3.5V` (Logic High)
* **Output Low Voltage:** Max `0.4V` (Logic Low)
* **Input High Voltage:** `1.6V* - 3.5V` (Logic High)
* **Input Low Voltage:** `-0.3V - 0.4V` (Logic Low)

> **CRITICAL HARDWARE NOTE:** The `RX` input signal of the A1M8 is recognized by current rather than pure voltage levels. To ensure reliable signal identification inside the internal system, the actual control node voltage of the `RX` pin will not drop lower than **1.6V**.

---

## 4. Power Supply and Consumption

The rover must isolate the core scanning digital system from the mechanical motor system using **separate power lines** to minimize ripple and ensure data accuracy.

```
                  ┌──────────────────┐
── 4.9V-5.5V DC ─►│  Scanner Core    │ (Low Ripple < 1%)
                  └──────────────────┘
                  ┌──────────────────┐
── 5.0V-10.0V DC ►│  Motor System    │ (High Current)
                  └──────────────────┘
```

### Detailed Electrical Specifications

| Parameter | Unit | Min | Typical | Max | Critical Constraints / Comments |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Scanner Core Voltage** | V | `4.9` | `5.0` | `5.5` | Voltages exceeding `5.5V` will permanently damage the core. |
| **Scanner Voltage Ripple** | mV | — | `20` | `50` | High ripple causes internal processing/working failures. |
| **Scanner Startup Current**| mA | — | `50` | `600` | Underpowering will result in initialization/boot loops. |
| **Scanner Current (Sleep)**| mA | — | `80` | `100` | At 5V input |
| **Scanner Current (Work)** | mA | — | `300` | `350` | At 5V input |
| **Motor System Voltage** | V | `5.0` | `5.0` | `10.0` | Adjust voltage to directly tune the physical scan rate. |
| **Motor System Current** | mA | — | `100` | — | At 5V input |

---

## 5. Physical Pinout & Connector Types

### Connector Configurations
* **Development Kit Base:** Uses a **PH2.54 - 7P** pitch connector block.
* **Production/Batch Version Connectors:**
    * *Motor Interface:* **PH1.25 - 3P** horizontal pitch connector.
    * *Core Interface:* **PH1.25 - 4P** vertical pitch connector.

### External Interface Signal Definitions

| Interface | Signal Name | Type | Description | Operating Range |
| :--- | :--- | :--- | :--- | :--- |
| **Motor Interface** | `VMOTO` | Power | Power line for the RPLIDAR A1 rotation motor | `5V - 9V` (Typical) |
| | `MOTOCTL` | Input | Enable signal / PWM Speed Control signal for the motor | `0V - VMOTO` |
| | `GND` | Power | Ground connection for the motor | `0V` |
| **Core Interface** | `VCC_5` | Power | Power supply line for the Range Scanner Core logic | `4.9V - 6V` (`5V` Typ.) |
| | `TX` | Output | Serial Transmission line from Range Scanner Core | `0V - 5V` |
| | `RX` | Input | Serial Reception line to Range Scanner Core | `0V - 5V` |
| | `GND` | Power | Ground connection for the Range Scanner Core | `0V` |

---

## 6. Built-in Safety & Fault Detection

The firmware integration architecture must constantly poll or check for health state indicators via the communication interface. The RPLIDAR A1 contains an embedded automatic safety shutdown circuit.

The LIDAR will instantly **shut down its laser and stop mechanical rotation** if any of the following failures are detected:
1. Laser transmission power exceeds the maximum eye-safety limit (`>5mW`).
2. Laser diode fails to power on normally.
3. Scanning speed of the Laser scanner subsystem becomes unstable.
4. Scanning speed is too slow/stalled.
5. Laser signal sensor array returns abnormal readings.

*Recovery Routine:* The host navigation architecture must intercept this state, query the status packet via the serial interface, and execute a soft/hard restart command sequence to recover from transient faults.

---

## 7. Software Development Context

* **Scan Data Packets:** Each single sample point data frame transmitted via UART contains the following contiguous variables:
    * `Distance` (measured in `mm` relative to the rotating center core).
    * `Heading` (measured in `degrees` from the forward baseline).
    * `Quality` (reported as an integer field indicating reflection strength level).
    * `Start Flag` (Boolean value signaling the initialization boundary of a brand-new 360-degree frame sweep).
* **Scan Rate Tuning:** The host computer can read the precise operational speed via telemetry and feed dynamic PWM adjustment commands to `MOTOCTL` to stabilize the target scan frequency (typically standardizing at `5.5 Hz`).
