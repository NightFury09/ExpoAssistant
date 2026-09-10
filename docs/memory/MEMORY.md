# Memory Index

- [micro-ROS version must match ROS distro](feedback_microros_version.md) — kilted library silently breaks subscriptions when using Humble agent; pin to `#humble` branch
- [RMCS-2303 stop command is 0x0000, not 0x0100](project_rmcs2303_stop_command.md) — 256 only selects Mode 1 and leaves the motor running; zero speed first, then write 0
- [Random motor direction was bad encoder feedback, not drivers](project_direction_rootcause.md) — resolved; motors test healthy; do NOT buy RMCS-2301; old fault report is outdated
- [RPLIDAR is an A1M8 — Sensitivity mode is invalid](project_rplidar_a1_scan_mode.md) — error 80008004 is a wrong scan_mode, not broken hardware; use Standard
- [USB [Errno 5] EIO = bad data link, not software](project_usb_eio_bad_link.md) — LED/power working proves nothing; run the port-open test before debugging ROS
- [Odometry now uses real encoders, not commanded RPM](project_odometry_real_encoders.md) — cpr=2737, radius=0.05, sep=0.4495; left encoder sign is negated
- [Parked rover work (D455 IMU, lidar riser, drive registers)](project_parked_work.md) — deferred 2026-09-07; details in NEXT_STEPS.md
- ["Nav2 broken" was really the ESP32 dropping commands](project_esp32_loop_starvation.md) — blocking Modbus starved micro-ROS; echo /rover_diag before tuning autonomy
