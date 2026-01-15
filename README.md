# Robotcraft
# 🤖 Autonomous Mobile Robot with LiDAR, Arduino & Raspberry Pi 4 (ROS)

This repository documents the design and implementation of an **autonomous mobile robot** integrating a 2D **LiDAR sensor**, an **Arduino** microcontroller for low-level control, and a **Raspberry Pi 4** running **ROS** (Robot Operating System) for high-level processing and autonomous navigation.
![WhatsApp Image 2025-12-13 at 16 54 57](https://github.com/user-attachments/assets/b6be09f5-387a-4c1b-9971-9da9ac684edf)
![WhatsApp Image 2025-12-13 at 16 57 34](https://github.com/user-attachments/assets/c47ef7c8-97bf-48a9-a2b1-e86d9781deff)

## 🔧 Hardware Architecture

- **2-wheeled differential drive chassis + caster wheel**
- **DC motors** with quadrature encoders
- **2D LiDAR** for obstacle detection and mapping
- **Arduino Uno** (or Mega) responsible for:
  - Motor control via PWM
  - Encoder signal processing
  - Serial communication with the Raspberry Pi
- **Raspberry Pi 4 (4 GB)** running:
  - ROS nodes for sensor data handling and control
  - SLAM for real-time mapping
  - Autonomous navigation and path planning
  - Visualization via RViz

## 🧠 Software Functionality

### 📦 Arduino Side
- PWM-based motor speed and direction control
- Encoder tick counting
- Serial communication with Raspberry Pi (USB or UART)

### 🐧 Raspberry Pi / ROS Side (Recommended: ROS Noetic + Ubuntu 20.04)
- **ROS Nodes**:
  - `lidar_publisher`: publishes LiDAR scan data
  - `encoder_reader`: reads encoder data via serial
  - `motor_command_sender`: sends velocity commands to Arduino
- **Navigation Stack** (optional):
  - SLAM using GMapping or Cartographer
  - Path planning with `move_base`
- **RViz**: for live 2D/3D visualization

## 🔌 Arduino ↔️ Raspberry Pi Communication

- Via **USB Serial** or **UART (GPIO pins)**
- Simple text-based protocol:
  - Example: `M100,-100\n` → sets left/right motor speeds
  - Response: `ENC,1234,1200\n` → returns encoder tick counts


## 📚 Dependencies

- **ROS Noetic**
- **rplidar_ros** (or compatible LiDAR driver)
- **serial** (ROS package for serial communication)
- **Encoder** library (Arduino)
- **TimerOne** (Arduino)

## 🚀 Getting Started

1. Flash the Arduino firmware from the `/arduino` folder
2. Set up and build the ROS workspace:
   ```bash
   cd ~/ros_ws
   catkin_make
   source devel/setup.bash
   roslaunch launch/bringup.launch
👤 Author
This project was developed as part of a personal robotics initiative and/or during the RobotCraft Summer Program by Ingeniarius (Portugal).

GitHub: zeroxy21



