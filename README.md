# TRIDENT — Transformable Robotic Independent Device for Extinguishing and Navigating Terrain

TRIDENT is an autonomous “fire‑fighting” robot that navigates a grid, detects “fires” via IR beacons on building tiles, and extinguishes them by deploying a ladder to break the beam on top of the building. The project includes a full Arduino Mega implementation with A* pathfinding, PID‑stabilized motion using a gyroscope, IR/QTI sensing, and a remote‑control mode for manual operation.

## Features
- **Two modes:** Autonomous scan‑and‑extinguish, and joystick‑based remote control.  
- **Pathfinding:** A* with Manhattan heuristic and obstacle avoidance on a padded grid.  
- **Motion control:** Gyro‑based heading hold with PID for straight runs and precise turns; line‑based calibration.  
- **Fire handling:** IR detection, building neighbor sweep, and servo‑driven ladder engagement.  
- **Debugging aids:** 7‑segment display status and LED indicators.

## Repository layout
```
/trident_autonomous/     # Code F1: final autonomous functionality (Arduino sketch)
/trident_remote/         # Code F2: remote‑control vehicle + base‑station sketches
/hardware/               # Schematics, wiring notes, laser‑cut drawings
/docs/                   # Report PDFs, figures, and diagrams
```

## Setup & wiring
1. Assemble the chassis and layered frame; mount sensors front/left/right at the top layer; mount the QTI pair under the front; mount the MPU‑6050 at the center.  
2. Wire according to the TRIDENT schematic (pin map per Appendix diagrams).  
3. Power: 4×AA for drive/ladder servos; 9V for the Arduino and low‑power peripherals.

## Building & flashing
- **Autonomous mode:** open `trident_autonomous/trident_autonomous.ino` in the Arduino IDE → select **Arduino Mega 2560** → Upload.  
- **Remote mode:**  
  - Upload `trident_remote/vehicle.ino` to the robot Mega.  
  - Upload `trident_remote/base_station.ino` to the base‑station Arduino used with the RF joystick transmitter.

## Calibration (run once per session)
1. Place the robot aligned with the grid; power on.  
2. Keep the robot still during gyro offset calibration.  
3. The robot performs line‑to‑line timing to set tile distances and centers itself before starting the search path.

## Key parameters to tune
- PID gains for straight‑line hold and turning tolerance.  
- QTI thresholds (surface‑dependent).  
- IR detection timeout and neighbor‑sweep dwell times.

## Troubleshooting
- **Drift or sloppy turns:** re‑do gyro offsets; lower speed cap; re‑tune PID.  
- **Missed lines:** adjust QTI thresholds; ensure sensors are ~2–3 mm above surface.  
- **IR false negatives:** verify sensor alignment, supply stability, and timing windows.  
- **Ladder hits too hard:** confirm ladder travel angle and paper‑tip compliance.

## License & citation
- Educational use under your chosen license. Please cite the project report.
