# IR2136 — Aerial & Underwater Robots (2025–2026)

This repository contains course work for **IR2136: Aerial & Underwater Robots**, with an emphasis on **end-to-end system integration**: simulation → GCS/HMI → networking/protocols → ROS integration → real vehicle.

## Contents
- **Lab 1** — Basic simulation with Ardupilot + QGroundControl (Live demo, plan uploaded)
- **Lab 2** — Basic simulation with Ardupilot + Python script + MAVLink (Uploaded)
- **Lab 3** — Basic simulation with Ardupilot + ROS2 + MAVlink (Uploaded)
- **Final Project** — In progress (**3D Gaussian Splatting / 3DGS**)

## Toolchain (course stack)
- **ArduPilot / ArduSub** as autopilot firmware
- **SITL** for simulation
- **QGroundControl (QGC)** as Ground Control Station
- **MAVLink** (typically over **UDP**, e.g. `14550`) for telemetry/commands
- **ROS 2** for distributed robotics logic and higher-level control

## Typical workflow
1. Run **SITL** and connect **QGC** via UDP.
2. Inspect MAVLink traffic (e.g., Wireshark) and automate using **PyMAVLink**.
3. Bridge/control from **ROS 2** (topics/services) while keeping safety/failsafes on the autopilot.
4. Transition to hardware once the pipeline is stable in simulation.

## Repo hygiene (important)
Do **not** commit build artifacts or logs (they bloat the repo and break reproducibility):
- Ignore: `build/`, `install/`, `log/`, binaries, large datasets/videos
- If you truly need large files, use **Git LFS** (but prefer linking/downloading scripts).

---
Maintainer: Pau Montagut-Bofi
