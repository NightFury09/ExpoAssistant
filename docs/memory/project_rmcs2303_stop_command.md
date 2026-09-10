---
name: rmcs2303-stop-command
description: "RMCS-2303 stop command is 0x0000, NOT 0x0100 — 256 only selects Mode 1 and leaves the motor running"
metadata: 
  node_type: memory
  type: project
  originSessionId: 94bb74df-26e3-4426-a987-fcd3fa136a8d
---

On the rover's RMCS-2303 motor drivers, writing 0x0100 (256) to control register 2 is "Mode 1 select" — it does NOT stop a running motor. The actual stop command is 0x0000, per the manual at `rover_project/rmcs_2303_manual.md` (section 5.2). Always write REG_SPEED=0 first, then 0x0000 to register 2.

**Why:** This caused weeks of misdiagnosis — firmware "disabled" motors with 256 and they kept spinning; the red LED (a normal run indicator) was blamed instead.

**How to apply:** Any firmware touching these drivers must use CTRL_STOP=0. The old `esp32_rover_firmware` (v1) and its archive still contain the wrong CTRL_DISABLE=256 — do not copy from them.
