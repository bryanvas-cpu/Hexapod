# Hexapod System – Replication Guide

This document describes the complete procedure to replicate the **Hexapod Hardware and Simulation Stack**, including SD card flashing, mechanical assembly, firmware upload, ROS 2 workspace setup, and simulation bring-up.

Follow the steps **in order**. Skipping steps may result in hardware damage or non-functional software.

---

## A) Hardware Replication

---

## 1. SD Card Preparation (Raspberry Pi 4)

> ⚠️ **CRITICAL WARNING – DATA LOSS RISK**  
> The flashing process will **erase the entire target device**.  
> Selecting the wrong device (e.g., your system SSD) will cause **irreversible data loss**.

### Requirements
- **SD Card**: 32 GB (minimum), Class 10 recommended
- **Host OS**: Linux
- **Reader**: USB SD card reader

### Preconditions
Before proceeding, ensure that:
- The SD card is **fully formatted**
- There are **no existing partitions**
- You have correctly identified the SD card device (`/dev/sdX`)

Verify using:
```bash
lsblk
```
### Flashing the Image

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

## 2. Mechanical Hardware
---
### Fabrication Files

- STL Files (3D Printing): 👉 Link

- DXF Files (Laser / Water-Jet Cutting): 👉 Link
  - Material: Aluminium 6061-T6
  - Thickness: 3 mm

ℹ️ Maintain dimensional accuracy during fabrication. Small tolerances can significantly affect leg kinematics and servo load.

## 3. Actuation Hardware
---
### Servos

- Quantity: 18
- Model: Waveshare ST3215 Smart Servo
- Link: https://www.waveshare.com/st3215-servo.htm

### Servo Driver

- Model: Waveshare ESP32 Servo Driver
- Link: https://www.waveshare.com/servo-driver-with-esp32.htm

### Documentation

- Official documentation for both the servo and the driver must be followed.
- Calibration, wiring, and power limits are defined there.

## 4. Servo Calibration (MANDATORY)
---
> ⚠️ DO NOT ASSEMBLE BEFORE CALIBRATION

- Before mounting any servo:
- Perform zero-position calibration for all 18 servos

> Verify:
> - Direction of rotation
> - Smooth motion
> - No jitter or abnormal behavior

#### ❗ Mounting uncalibrated servos can cause: Mechanical collisions, Gear damage and Structural deformation

## 5. ESP32 Firmware Upload
---
#### Firmware Path:
```bash
src/hexapod_firmware/Arduino/final_code_esp_serial/
└── final_code_esp_serial.ino
```
### Steps for Upload of ESP32 code

- Install Arduino IDE
- Install ESP32 board support
- Install all required libraries as specified in the Servo Driver Wiki
- Connect ESP32 via USB
Select correct: Board and Port
Upload the firmware

> ⚠️ Ensure stable power to the servo driver during flashing and testing.

## 6. Pre-Assembly Validation
---
Before mounting servos onto the structure:

- Power the system
- Confirm:
  - Correct angular response
  - No overheating
  - No unexpected current draw

🔧 Fixing issues after assembly is significantly harder and riskier.

## 7. Final Assembly & Control
- Assemble the hexapod mechanically
- Ensure Proper Orientation, and ID of Servos while Mounting at appropriate locations.
- Ensure proper cable routing and strain relief
- Insert SD card into Raspberry Pi 4
- Power on the system
- Connect the USB dongle of the controller to the Raspberry Pi
- Turn on the controller
- control the robot via the joystick

🕹️ Perform first power-on tests with the robot lifted off the ground.