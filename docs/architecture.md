# Firmware Architecture Overview

## System Role

The ESP32-S3 is a **self-contained differential-drive controller**. It
receives `cmd_vel` from a master computer (ROS 2), performs diff-drive
kinematics (linear.x + angular.z → left/right wheel speeds), commands
the motor controller over CAN, and computes + publishes odometry from
wheel feedback.

The motor backend is selected at build time behind a hardware-agnostic
HAL (`main/motor_driver.h`). The **current robot hardware** is a single
**ZLAC8015D** dual-channel servo drive (CANopen / CiA 402) driving two
**ZLLG65ASM250 V3.0** 6.5″ direct-drive hub motors. A legacy **VESC**
backend (2× controllers, custom CAN) remains selectable for older builds.

```
┌──────────────┐    micro-ROS     ┌──────────────────┐   CANopen/CiA402  ┌──────────────┐
│  Master PC   │◄────────────────►│   ESP32-S3       │◄─────────────────►│  ZLAC8015D   │
│  (ROS 2)     │   USB-CDC        │   diff-drive     │   TWAI 500 kbit/s  │  L ch ─ hub  │
│              │                  │   controller     │                   │  R ch ─ hub  │
│  cmd_vel ──► │                  │                  │                   └──────────────┘
│  ◄── odom    │                  │   kinematics     │
│  ◄── status  │                  │   odometry       │
└──────────────┘                  └──────────────────┘
                                         ▲
                                         │ PWM (failsafe)
                                    ┌────┴─────┐
                                    │ RC Rx    │
                                    └──────────┘
```

## Hardware

**Waveshare ESP32-S3-RS485-CAN** (SKU 32154) — used outside DIN-rail
enclosure to access the full 20-pin GPIO header.

| Resource       | Detail                                            |
|----------------|---------------------------------------------------|
| MCU            | ESP32-S3R8, dual-core LX7 @ 240 MHz               |
| Flash / PSRAM  | 16 MB / 8 MB                                       |
| CAN            | Isolated, TVS-protected, 120 Ω term. jumper        |
| RS485          | Isolated, TVS/ESD protected (unused for now)       |
| Power          | 7–36 V DC or USB-C 5 V                             |
| Wireless       | Wi-Fi 2.4 GHz, BLE 5                               |

### Drivetrain (current hardware)

| Resource        | Detail                                                     |
|-----------------|------------------------------------------------------------|
| Motor controller| **ZLAC8015D V4** — 1× dual-channel hub-servo drive, CANopen|
| Protocol        | CANopen CiA 402, Profile Velocity mode, 500 kbit/s         |
| Node / channels | Single node (default ID 1); LEFT = sub-index 1, RIGHT = 2  |
| Motors          | 2× **ZLLG65ASM250 V3.0** 6.5″ direct-drive hub motors      |
| Motor spec      | 24 VDC, 15 pole-pairs, 4096-line encoder, 170 mm wheel     |
| Speed           | Rated 500 r/min, peak 560 r/min (direct drive, 1:1)        |

Per-axis objects map LEFT→sub-index 1 and RIGHT→sub-index 2. Statusword
0x6041 packs both axes in one U32 (**high 16 = LEFT, low 16 = RIGHT**).
Backend details and object-dictionary assumptions are documented in
`main/motor_driver_zlac8015d.c`.

## Core Allocation

| Core | Responsibilities                                       | Priority |
|------|--------------------------------------------------------|----------|
| 0    | micro-ROS agent, DDS middleware, USB-CDC, odometry pub | Normal   |
| 1    | CAN (TWAI) TX/RX, RC PWM input, failsafe, kinematics  | High     |

Real-time CAN and PWM processing is isolated on Core 1 to avoid jitter
from the networking stack. Diff-drive kinematics (cmd_vel → wheel speeds)
runs on Core 1 so motor commands are computed and sent with minimal
latency. Odometry is computed from wheel feedback on Core 1 and published
via micro-ROS on Core 0.

## FreeRTOS Tasks (planned)

| Task              | Core | Rate       | Purpose                              |
|-------------------|------|------------|--------------------------------------|
| `uros_task`       | 0    | ~10 ms     | micro-ROS spin, pub/sub, odom pub    |
| `motor_task`      | 1    | ~20 ms     | Kinematics + arbitration → HAL set   |
| `zlac_tx` (backend)| 1   | ~20 ms     | RPDO controlword + target velocities |
| `rc_failsafe_task`| 1    | ~20 ms     | Read RC PWM, arcade mix, failsafe    |

## ROS 2 Interface (planned)

| Direction | Topic / Service           | Type                    | Purpose                    |
|-----------|---------------------------|-------------------------|----------------------------|
| Sub       | `cmd_vel`                 | `geometry_msgs/Twist`   | Drive commands (v, omega)  |
| Pub       | `odom`                    | `nav_msgs/Odometry`     | Wheel odometry             |
| Pub       | `motor/status`            | TBD                     | Motor telemetry (V, A, T)  |
| Pub       | `failsafe/active`         | `std_msgs/Bool`         | Failsafe state             |

## Key Design Decisions

Documented as Architecture Decision Records in [`adr/`](adr/).

| ADR   | Title                              | Status   |
|-------|------------------------------------|----------|
| 0001  | Core allocation strategy           | Proposed |
| 0002  | micro-ROS transport                | Proposed |
| 0003  | VESC CAN protocol                  | Superseded by 0010 |
| 0004  | Diff-drive kinematics on ESP32     | Proposed |
| 0005  | Odometry computation               | Proposed |
| 0006  | RC failsafe behavior & mixing      | Proposed |
| 0007  | CAN bus topology & termination     | Proposed |
| 0008  | Debug console on SH1.0 UART        | Proposed |
| 0010  | ZLAC8015D CANopen migration        | Accepted |

## Building & Flashing

### Prerequisites

- **ESP-IDF v5.4** — [installation guide](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/get-started/)
- **Python packages** for micro-ROS build:
  ```bash
  pip install catkin_pkg lark empy==3.3.4
  ```

### Clone

```bash
git clone --recursive https://github.com/BigMuzzy/esp32-s3-uros-controller.git
cd esp32-s3-uros-controller/firmware
```

If already cloned without `--recursive`:
```bash
git submodule update --init --recursive
```

### Build

```bash
source ~/esp/esp-idf/export.sh
idf.py build
```

The first build takes several minutes — the micro-ROS component
downloads and cross-compiles the ROS 2 middleware. Subsequent builds
are incremental.

### Flash

Connect the board via USB-C, then:

```bash
idf.py -p /dev/ttyACM0 flash
```

### Monitor

```bash
idf.py -p /dev/ttyACM0 monitor
```

**Note:** In production the USB-C port is used by the micro-ROS
agent (USB-CDC transport). Serial monitor output goes to UART0
or should be disabled. During development, monitor and micro-ROS
agent cannot use the same port simultaneously.

### Build, Flash & Monitor (combined)

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

### Menuconfig

```bash
idf.py menuconfig
```

Relevant settings under `micro-ROS Settings` and `Component config → TWAI`.

## Source Layout

```
firmware/
├── main/
│   ├── main.c                      # app_main — task creation & init
│   ├── uros_task.h / .c            # micro-ROS spin loop
│   ├── motor_driver.h              # hardware-agnostic motor HAL contract
│   ├── motor_task.h / .c           # Core 1 control loop (HAL + failsafe)
│   ├── diff_drive.h / .c           # kinematics & odometry
│   ├── rc_failsafe.h / .c          # RC PWM + arcade mixing + failsafe
│   ├── canopen.h / .c              # CiA 301 master (ZLAC backend)
│   ├── cia402.h / .c               # CiA 402 drive state machine
│   ├── motor_driver_zlac8015d.*    # ZLAC8015D CANopen backend (default)
│   ├── vesc_can.* / motor_driver_vesc.*  # legacy VESC backend
│   └── tune_cli.* / tune_transport_*     # VESC PID tuning CLI
├── components/
│   └── micro_ros_espidf_component/
├── CMakeLists.txt
└── sdkconfig
```

The active backend is chosen with `idf.py menuconfig → Motor Driver →
Motor controller backend` (default **ZLAC8015D**). CMake compiles only
the selected backend's sources.
