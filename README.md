# Pit Detection Safety System for Mobile Robots

## Overview

This project implements a pit and downward-step detection safety system for a mobile robot using four IR sensors connected to an Arduino Nano.

The IR sensors are mounted at the four corners of the robot and continuously monitor the presence of ground beneath the robot. If any sensor detects a pit, stair edge, or downward step, the Arduino Nano immediately reports the hazard to an ESP32-S3 through I²C communication.

The ESP32-S3 acts as the communication bridge between the sensor subsystem and the ROS 2 Jazzy navigation stack running on a Raspberry Pi.

A ROS 2 safety node (`pit_safety_node`) receives the pit detection information and overrides the navigation commands by publishing a zero-velocity command on `/cmd_vel_out`, ensuring that the robot stops before moving into a dangerous area.

---

## Features

- Four-corner pit detection.
- Downward-step detection.
- Arduino Nano-based sensor monitoring.
- I²C communication between Arduino Nano and ESP32-S3.
- micro-ROS integration.
- ROS 2 Jazzy compatible.
- Automatic emergency stop.
- Safety hold timer after pit clearance.
- Suitable for mobile robots and semi-humanoid robots.

---

## Hardware Requirements

- Arduino Nano
- ESP32-S3
- Raspberry Pi
- 4 × IR Sensors
- Motor Controller
- Mobile Robot Platform

---

## Hardware Connections

### IR Sensors to Arduino Nano

| Sensor Position | Arduino Pin |
|----------------|------------|
| Front Left (FL) | D2 |
| Front Right (FR) | D3 |
| Rear Left (BL) | D4 |
| Rear Right (BR) | D5 |

### Sensor Power Connections

| IR Sensor Pin | Arduino Nano |
|--------------|-------------|
| VCC | 5V |
| GND | GND |
| OUT | D2/D3/D4/D5 |

Sensor Logic:

- LOW → Ground detected (Safe)
- HIGH → Pit/Step detected

---

### Arduino Nano to ESP32-S3 (I²C)

| Arduino Nano | ESP32-S3 |
|-------------|----------|
| A4 (SDA) | GPIO8 (SDA) |
| A5 (SCL) | GPIO9 (SCL) |
| GND | GND |

I²C Configuration:

- Arduino Nano → Slave
- ESP32-S3 → Master
- Slave Address → `0x08`

---

### ESP32-S3 to Raspberry Pi

The ESP32-S3 communicates with the Raspberry Pi through micro-ROS serial transport.

Example micro-ROS Agent:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

---

## System Architecture

```text
IR Sensors (4)
      │
      ▼
Arduino Nano
(I²C Slave)
      │
      ▼
ESP32-S3
(I²C Master + micro-ROS)
      │
      ▼
Raspberry Pi
ROS 2 Jazzy
pit_safety_node
      │
      ▼
cmd_vel_out = 0
      │
      ▼
Motor Controller
      │
      ▼
Robot Stops
```

---

## Sensor Placement

```text
Front

[FL]         [FR]


[BL]         [BR]

Rear
```

- FL → Front Left
- FR → Front Right
- BL → Rear Left
- BR → Rear Right

---

## Working Principle

1. Four IR sensors continuously monitor the ground beneath the robot.
2. Arduino Nano reads all sensor states.
3. If any sensor detects a pit or downward step, a hazard flag is generated.
4. The hazard information is sent to the ESP32-S3 using I²C.
5. ESP32-S3 publishes the detection information to ROS 2 through micro-ROS.
6. The ROS 2 safety node receives the pit information.
7. The safety node overrides the navigation velocity command.
8. A zero-velocity command is published to `/cmd_vel_out`.
9. The robot stops immediately.

---

## ROS 2 Topics

### Subscriptions

| Topic | Message Type |
|---------|--------------|
| `/cmd_vel` | geometry_msgs/Twist |
| `/pit_detected` | std_msgs/Bool |
| `/pit_status` | std_msgs/UInt8 |

### Publications

| Topic | Message Type |
|---------|--------------|
| `/cmd_vel_out` | geometry_msgs/Twist |
| `/pit_override` | std_msgs/Bool |

---

## Safety Behaviour

| Condition | Action |
|-----------|---------|
| All sensors detect ground | Robot moves normally |
| Any sensor detects pit | Robot stops |
| Any sensor detects downward step | Robot stops |
| Pit cleared | Resume after safety hold timer |

---

## Software Stack

- Arduino IDE
- ESP32 Arduino Framework
- micro-ROS
- ROS 2 Jazzy
- Raspberry Pi OS

---

## Future Improvements

- Obstacle avoidance integration
- Sensor fusion using ToF sensors
- Recovery behaviours
- Autonomous path replanning
- Visual and audio warning systems

---

## ROS 2 Safety Node

The `pit_safety_node` acts as a velocity gate between the navigation stack and the motor controller.

When a pit is detected:

- Incoming `/cmd_vel` commands are blocked.
- A zero Twist message is published on `/cmd_vel_out`.
- The robot remains stopped until the pit is cleared and the configured hold timer expires.

This ensures safe operation near stairs, platform edges, and sudden drops.
