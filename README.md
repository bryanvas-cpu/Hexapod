# Hexapod System – Replication Guide

This document describes the complete procedure to replicate the **Hexapod Hardware and Simulation Stack**, including SD card flashing, mechanical assembly, firmware upload, ROS 2 workspace setup, and simulation bring-up.

Follow the steps **in order**. Skipping steps may result in hardware damage or non-functional software.

---

## A) Hardware Replication

---

### 1. SD Card Preparation (Raspberry Pi 4)

> ⚠️ **CRITICAL WARNING – DATA LOSS RISK**  
> The flashing process will **erase the entire target device**.  
> Selecting the wrong device (e.g., your system SSD) will cause **irreversible data loss**.

#### Requirements
- **SD Card**: 32 GB (minimum), Class 10 recommended
- **Host OS**: Linux
- **Reader**: USB SD card reader

#### Preconditions
Before proceeding, ensure that:
- The SD card is **fully formatted**
- There are **no existing partitions**
- You have correctly identified the SD card device (`/dev/sdX`)

Verify using:
```bash
lsblk
```
#### Flashing the Image

- Clone the provided image onto the SD card using:
- Image File: add link here

```bash
sudo dd if=/home/path_to_img_file/ubuntu_pi_backup.img \
        of=/dev/sdX \
        bs=4M \
        status=progress \
        conv=fsync
```
After completion:
```bash
sync
```

### 2. Mechanical Hardware
---
#### Fabrication Files

- STL Files (3D Printing): [STL](STL)

- DXF Files (Laser Cutting): [DXF](DXF)
  - Material: Aluminium 6061-T6
  - Thickness: 2 mm

ℹ️ Maintain dimensional accuracy during fabrication. Small tolerances can significantly affect leg kinematics and servo load.

### 3. Actuation Hardware
---
#### Servos

- Quantity: 18
- Model: Waveshare ST3215 Smart Servo
- Link: https://www.waveshare.com/st3215-servo.htm

#### Servo Driver

- Model: Waveshare ESP32 Servo Driver
- Link: https://www.waveshare.com/servo-driver-with-esp32.htm

#### Documentation

- Official documentation for both the servo and the driver must be followed.
- Calibration, wiring, and power limits are defined there.

### 4. Servo Calibration (MANDATORY)
---
> ⚠️ DO NOT ASSEMBLE BEFORE CALIBRATION

- Before mounting any servo:
- Perform zero-position calibration for all 18 servos

> Verify:
> - Direction of rotation
> - Smooth motion
> - No jitter or abnormal behavior

##### ❗ Mounting uncalibrated servos can cause: Mechanical collisions, Gear damage and Structural deformation

### 5. ESP32 Firmware Upload
---
##### Firmware Path:
```bash
src/hexapod_firmware/Arduino/final_code_esp_serial/
└── final_code_esp_serial.ino
```
#### Steps for Upload of ESP32 code

- Install Arduino IDE
- Install ESP32 board support
- Install all required libraries as specified in the Servo Driver Wiki
- Connect ESP32 via USB
Select correct: Board and Port
Upload the firmware

> ⚠️ Ensure stable power to the servo driver during flashing and testing.

### 6. Pre-Assembly Validation
---
Before mounting servos onto the structure:

- Power the system
- Confirm:
  - Correct angular response
  - No overheating
  - No unexpected current draw

🔧 Fixing issues after assembly is significantly harder and riskier.

### 7. Final Assembly & Control
- Assemble the hexapod mechanically
- Ensure Proper Orientation, and ID of Servos while Mounting at appropriate locations.
- Ensure proper cable routing and strain relief
- Insert SD card into Raspberry Pi 4
- Power on the system
- Connect the USB dongle of the controller to the Raspberry Pi
- Turn on the controller
- control the robot via the joystick

🕹️ Perform first power-on tests with the robot lifted off the ground.

---
## B) Simulation Replication (ROS 2)

This section describes how to replicate and run the **Hexapod Simulation Stack** on a development machine.  
The simulation mirrors the real robot’s control architecture, kinematics, and joystick interface.

> ⚠️ **IMPORTANT**  
> Use a **native Ubuntu 22.04 installation (Dual Boot)**.  
> **Do NOT use a virtual machine**, as it can cause issues.

### 1. Host System Requirements
---

- **Operating System**: Ubuntu 22.04 LTS (Desktop recommended)
- **Installation Type**: Dual Boot / Bare Metal
- **Internet Connection**: Required
- **Controller**: PS4 or Xbox Controller (USB or Bluetooth)

### 2. Install ROS 2 Humble
---

Install ROS 2 Humble by following the official instructions:

🔗 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Ensure that:
- ROS 2 Humble is correctly sourced
- `ros2` command works in a new terminal

### 3. Create the Workspace and Clone Repository
---

Create the workspace directory:
```bash
mkdir ~/Hexapod
cd ~
git clone --no-checkout https://github.com/bryanvas-cpu/Hexapod.git
cd Hexapod
git sparse-checkout init --cone
git sparse-checkout set src
git checkout
```
### 4. Install Dependencies
---
```bash
cd ~/Hexapod
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the Workspace
---
```bash
cd ~/Hexapod
colcon build
```

### 6. Source the Workspace
---
After a successful build, source the workspace:
``` bash
source install/setup.bash
```

### 7. Launch the Simulation
---
```bash
ros2 launch hexapod_bringup simulated_robot.launch.py
```
- The robot model should spawn in simulation
- Controllers, TFs, and control nodes should start automatically

### 8. Controller Setup
---
- Connect a PS4 or Xbox controller to the host machine
- Via USB or Bluetooth
- Ensure the controller is detected by the OS
- The simulation should automatically accept joystick input

#### 🎮 You should now be able to control the simulated hexapod using the controller.

## C) Explanation of File Structure

This section provides an overview of the important directories and packages in the hexapod workspace, explaining their purpose and role in the overall system architecture.

---

### 1. [`~/Hexapod/src`](src)

This directory contains **all individual ROS 2 packages** that make up the hexapod software stack. Key packages are listed below.

---

#### a) [`hexapod_bringup`](src/hexapod_bringup)

- Contains all **launch files** for both **Simulation** and **Hardware**
- Launch files are located in the `launch/` directory
- Acts as the **single entry point** for bringing up the full system

---

#### b) [`hexapod_controller`](src/hexapod_controller)

This package contains the **core mathematical and control logic** of the hexapod. Each file is structured around a specific responsibility.

1. `inverse_kinematics.cpp`  
   - Transforms foot-tip positions from each leg’s **local Cartesian coordinate frame**  
   - Computes the corresponding **joint angles** for that leg

2. `body_pose_generator.cpp`  
   - ROS 2 node that reads **joystick-commanded target poses** and **IMU orientation**
   - Runs **PID control** on roll and pitch
   - Publishes **stabilized body pose commands** (RPY + XYZ) at a fixed rate

3. `tip_trajectory_generator.cpp`  
   - Converts **joystick velocity and gait commands** into **time-parameterized, Bezier-based foot trajectories**
   - Generates synchronized leg trajectories for all six legs
   - Publishes leg position targets in the **robot body frame**

4. `tip_pose_generator.cpp`  
   - Applies **body translation and rotation** to generated leg trajectories
   - Transforms foot positions into each leg’s **local coordinate frame**
   - Publishes final **foot-tip pose targets** for inverse kinematics

5. `imu_pose_generator.cpp`  
   - Used only when an **IMU is installed**
   - Not active on the current laboratory hardware setup

---

#### c) [`hexapod_description`](src/hexapod_description)

- Contains the **robot description files**
- Structure:
  - `urdf/` – Robot URDF files
  - `meshes/` – Associated STL meshes

Key launch files:
1. `display.launch.py`  
   - Visualizes the robot URDF in **RViz**

2. `gazebo.launch.py`  
   - Spawns and visualizes the hexapod in **Gazebo**

---

#### d) [`hexapod_firmware`](src/hexapod_description)

- Contains all firmware and hardware-interface-related code

Structure:
1. `Arduino/`  
   - ESP32 firmware code to be uploaded via Arduino IDE

2. `src/`  
   - Implements the **ROS 2 Control hardware interface**
   - Acts as the bridge between ROS 2 and the ESP32 servo driver

---

#### e) [`hexapod_kinematics`](src/hexapod_kinematics)

- Currently empty  
- Reserved for future kinematics-related expansions or refactoring

---

#### f) [`hexapod_utils`](src/hexapod_utils)

- Contains miscellaneous **utility functions and helper nodes**
- Supports multiple packages across the stack

---

#### g) [`serial_communicator`](src/serial_communicator)

- Implements the **serial communication protocol**
- Handles communication between:
  - ESP32 servo driver
  - Raspberry Pi (ROS 2 side)

---

### 2. [`STL`](STL)

- Contains all **STL files** required for **3D printing**
- Used for fabricating mechanical components of the hexapod

---