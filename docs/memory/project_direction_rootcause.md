---
name: direction-rootcause
description: "Rover's \"non-deterministic motor direction\" was bad encoder feedback, NOT a driver defect — resolved; do not buy new drivers"
metadata: 
  node_type: memory
  type: project
  originSessionId: 94bb74df-26e3-4426-a987-fcd3fa136a8d
  modified: 2026-08-31T08:33:13.392Z
---

The rover's long-running "teleop direction is random" problem was root-caused to **marginal encoder feedback** (wiring/connectors/motor), not the RMCS-2303 drivers. The RMCS-2303 is closed-loop and reads the encoder to resolve rotation direction; with bad feedback it latched direction randomly and reported impossible position values (28k–800k counts, jumps in multiples of 65536).

After the encoder issue was fixed, the `motor_test` health firmware (`-e health`, TEST_PHASE=6) showed BOTH motors PASS all criteria across 4 cycles: CW always +, CCW always −, ~2230 counts per 50 RPM / 2 s run (334 lines ×4 = 1336 counts/rev — correct scale), speed repeatable within ~2%.

**Why:** This nearly led to an unnecessary, expensive RMCS-2301 purchase. The RMCS-2301 is a step/direction drive but is ALSO closed-loop/encoder-dependent, so it would not have fixed an encoder-feedback fault.

**How to apply:** For any recurrence of erratic direction, suspect encoder wiring/connectors FIRST. Also watch power stability — the drivers repeatedly dropped off the Modbus bus (brownouts) during testing; ensure the driver supply holds both motors under load. See [[rmcs2303-stop-command]]. The earlier `drivetrain_fault_report.docx` is OUTDATED (concluded hardware replacement) — do not act on it.
