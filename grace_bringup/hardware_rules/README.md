# Hardware Setup Rules for GRACE Robot

This folder contains udev rules and hardware configuration documentation for deploying the GRACE robot on real hardware (Jetson Orin Nano).

## 📁 Files in This Folder

- `99-grace-robot.rules` - Main udev rules file for all hardware devices
- `README.md` - This file
- `HARDWARE_SETUP.md` - Complete hardware setup guide

## 🔌 Hardware Components

1. **RPLidar A2M7** - 2D Laser Scanner
2. **Arduino** - Motor controller interface
3. **MPU6050** - IMU sensor (I2C)

## ⚙️ Quick Setup

### 1. Install Udev Rules

```bash
# Copy udev rules to system
sudo cp 99-grace-robot.rules /etc/udev/rules.d/

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify (unplug/replug devices)
ls -l /dev/rplidar /dev/arduino
```

### 2. Configure I2C Permissions

```bash
# Add user to i2c group
sudo usermod -aG i2c $USER

# Set I2C device permissions
sudo chmod 666 /dev/i2c-*

# Reboot to apply group changes
sudo reboot
```

### 3. Test Hardware

```bash
# Test RPLidar
ros2 run rplidar_ros rplidar_node --ros-args --params-file $(ros2 pkg prefix grace_bringup)/share/grace_bringup/config/rplidar_a2m7.yaml

# Test Arduino connection
ls -l /dev/arduino
# Should show symlink to /dev/ttyACM0 or similar

# Test I2C (IMU)
i2cdetect -y 1
# Should show device at address 0x68
```

### 4. Launch Real Robot

```bash
ros2 launch grace_bringup real_robot.launch.py
```

## 🚨 Troubleshooting

### RPLidar Not Detected
- Check USB connection
- Verify udev rules: `ls -l /dev/rplidar`
- Check dmesg: `dmesg | grep ttyUSB`

### Arduino Not Responding
- Verify Arduino is programmed with correct firmware
- Check serial permissions: `sudo usermod -aG dialout $USER`
- Test: `ls -l /dev/arduino`

### IMU Not Found
- Check I2C wiring (SDA, SCL, VCC, GND)
- Verify I2C is enabled: `i2cdetect -y 1`
- Check address 0x68 appears in I2C scan

## 📖 Detailed Documentation

See `HARDWARE_SETUP.md` for complete hardware integration guide.
