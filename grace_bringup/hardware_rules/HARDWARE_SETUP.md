# GRACE Robot - Complete Hardware Setup Guide

This guide walks you through setting up the GRACE robot on real hardware (Jetson Orin Nano or similar Linux SBC).

---

## 📦 Hardware Requirements

### Main Components
- **Computer:** Jetson Orin Nano Super 8GB (or Ubuntu 22.04 Linux PC)
- **Motors:** 2x Hoverboard BLDC motors with encoders
- **Motor Controller:** Arduino Uno/Mega (with custom firmware)
- **Lidar:** Slamtec RPLidar A2M7
- **IMU:** MPU6050 (optional for sensor fusion)
- **Power:** 12V battery for motors, 5V for electronics

### Connections Overview
```
Jetson Orin Nano
├── USB → RPLidar A2M7 (/dev/rplidar)
├── USB → Arduino (/dev/arduino)
│   └── Arduino Controls:
│       ├── PWM → Hoverboard Motor Drivers → BLDC Motors
│       └── Digital Pins ← Wheel Encoders
└── I2C (GPIO) → MPU6050 IMU (address 0x68)
```

---

## 🔧 Step-by-Step Setup

### Phase 1: Operating System Setup

#### 1.1 Install Ubuntu 22.04 on Jetson
```bash
# Follow NVIDIA Jetson setup guide
# Ensure Ubuntu 22.04 (Jammy) is installed
lsb_release -a
# Should show: Ubuntu 22.04 LTS
```

#### 1.2 Install ROS 2 Humble
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

#### 1.3 Install Required ROS Packages
```bash
# RPLidar driver
sudo apt install -y ros-humble-rplidar-ros

# Nav2 stack
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# SLAM Toolbox
sudo apt install -y ros-humble-slam-toolbox

# Additional tools
sudo apt install -y ros-humble-robot-localization
sudo apt install -y ros-humble-twist-mux
```

---

### Phase 2: Hardware Connections

#### 2.1 Connect RPLidar A2M7
1. Plug RPLidar USB cable into Jetson
2. Check device appears:
   ```bash
   ls -l /dev/ttyUSB*
   # Should show: /dev/ttyUSB0 or similar
   ```

#### 2.2 Connect Arduino
1. Upload firmware from `grace_firmware/firmware/robot_control/` to Arduino
2. Connect Arduino USB to Jetson
3. Check device:
   ```bash
   ls -l /dev/ttyACM*
   # Should show: /dev/ttyACM0 or similar
   ```

#### 2.3 Wire Arduino to Motors
```
Arduino → Hoverboard Driver → BLDC Motor
Pin D9  → PWM Left Motor
Pin D10 → PWM Right Motor
Pin D2  → Left Encoder A
Pin D3  → Left Encoder B
Pin D4  → Right Encoder A
Pin D5  → Right Encoder B
```

#### 2.4 Connect MPU6050 IMU (Optional)
```
MPU6050 → Jetson I2C
VCC → 3.3V
GND → GND
SDA → GPIO Pin 3 (I2C_SDA)
SCL → GPIO Pin 5 (I2C_SCL)
```

Verify I2C:
```bash
sudo apt install -y i2c-tools
i2cdetect -y 1
# Should show device at 0x68
```

---

### Phase 3: Udev Rules Setup

#### 3.1 Install Udev Rules
```bash
cd ~/grace_ws/src/grace_bringup/hardware_rules

# Copy udev rules
sudo cp 99-grace-robot.rules /etc/udev/rules.d/

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to required groups
sudo usermod -aG dialout $USER
sudo usermod -aG i2c $USER

# IMPORTANT: Logout and login (or reboot) for group changes to take effect
```

#### 3.2 Verify Device Symlinks
```bash
# After unplugging and replugging devices
ls -l /dev/rplidar /dev/arduino /dev/i2c-*

# Should show:
# /dev/rplidar -> ttyUSB0
# /dev/arduino -> ttyACM0
# /dev/i2c-1 (with rw permissions)
```

---

### Phase 4: Build GRACE Workspace

#### 4.1 Clone/Copy GRACE Repository
```bash
mkdir -p ~/grace_ws/src
cd ~/grace_ws/src
# Copy your grace packages here
```

#### 4.2 Install Dependencies
```bash
cd ~/grace_ws
rosdep install --from-paths src --ignore-src -r -y
```

#### 4.3 Build Workspace
```bash
cd ~/grace_ws
colcon build --symlink-install
source install/setup.bash
```

#### 4.4 Add to .bashrc
```bash
echo "source ~/grace_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Phase 5: Hardware Testing

#### 5.1 Test RPLidar A2M7
```bash
# Terminal 1: Launch RPLidar node
ros2 run rplidar_ros rplidar_node --ros-args \
  --params-file ~/grace_ws/src/grace_bringup/config/rplidar_a2m7.yaml

# Terminal 2: Check scan data
ros2 topic hz /scan
# Should show: ~11.6 Hz

ros2 topic echo /scan --once
# Should show laser scan data
```

#### 5.2 Test Arduino Communication
```bash
# Check serial port
ls -l /dev/arduino

# Monitor Arduino serial output (optional)
sudo apt install -y screen
screen /dev/arduino 57600
# Should see encoder/motor debug messages if programmed
```

#### 5.3 Test IMU (if connected)
```bash
# Check I2C detection
i2cdetect -y 1
# Verify 0x68 shows up

# Launch IMU driver
ros2 run grace_firmware mpu6050_driver.py

# Check IMU data
ros2 topic echo /imu/out --once
```

---

### Phase 6: Launch Real Robot

#### 6.1 Full System Launch
```bash
# Launch everything
ros2 launch grace_bringup real_robot.launch.py

# Verify all nodes running
ros2 node list
```

Expected nodes:
- `/rplidar_node` - Lidar driver
- `/controller_manager` - ros2_control
- `/grace_controller` - Diff drive controller
- `/robot_state_publisher` - TF publisher
- `/mpu6050_driver` - IMU driver

#### 6.2 Test Manual Control
```bash
# Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/grace_controller/cmd_vel
```

#### 6.3 Check TF Tree
```bash
ros2 run tf2_tools view_frames
# Should generate frames.pdf showing: odom → base_footprint → base_link → wheels/lidar
```

---

### Phase 7: SLAM and Navigation

#### 7.1 Create Map
```bash
# Launch with SLAM
ros2 launch grace_bringup real_robot.launch.py use_slam:=true

# Drive around to build map (use keyboard teleop)

# Save map
cd ~/grace_ws/src/grace_mapping/maps
ros2 run nav2_map_server map_saver_cli -f my_real_map
mkdir my_real_map
mv my_real_map.* my_real_map/
cd my_real_map
mv my_real_map.pgm map.pgm
mv my_real_map.yaml map.yaml
```

#### 7.2 Autonomous Navigation
```bash
# Launch with your map
ros2 launch grace_bringup real_robot.launch.py map_name:=my_real_map

# In RViz:
# 1. Set "2D Pose Estimate" for initial localization
# 2. Send "Nav2 Goal" for autonomous navigation
```

---

## 🔧 Calibration and Tuning

### Motor Controller Calibration
After initial testing, you may need to calibrate:

1. **Wheel Diameter** - Measure actual wheel radius
   ```bash
   # Update in: grace_controller/config/grace_controllers.yaml
   wheel_radius: 0.08255  # Adjust based on measurement
   ```

2. **Wheelbase** - Measure distance between wheel centers
   ```bash
   wheel_separation: 0.45468  # Adjust based on measurement
   ```

3. **Odometry Validation** - Test straight line and rotation
   ```bash
   # Drive 1 meter forward, measure actual distance
   # Rotate 90°, measure actual angle
   # Adjust wheel parameters if error > 5%
   ```

### Lidar Mounting
- Ensure lidar is level (horizontal)
- Verify lidar height matches URDF: `laser_joint` z-offset
- Test 360° visibility (no blind spots from robot body)

### IMU Calibration (if using sensor fusion)
```bash
# The MPU6050 driver may need bias calibration
# Place robot still on level surface
# Record accelerometer/gyro readings
# Adjust offsets in mpu6050_driver.py if needed
```

---

## 🚨 Common Issues

### RPLidar Connection Fails
**Symptom:** `Failed to bind communication port`

**Solutions:**
1. Check USB cable connection
2. Verify baudrate: `256000` for A2M7
3. Check permissions: `ls -l /dev/rplidar` should show rw permissions
4. Test manually:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000
   ```

### Arduino Not Responding
**Symptom:** Controllers fail to load, motors don't move

**Solutions:**
1. Verify Arduino firmware is uploaded
2. Check serial connection: `screen /dev/arduino 57600`
3. Test with simple sketch to verify USB communication
4. Ensure baud rate matches firmware (usually 57600)

### Controllers Won't Activate
**Symptom:** `Failed to activate controller_manager`

**Solutions:**
1. Check ros2_control plugin in URDF: `/dev/arduino` path correct
2. Verify Arduino is responding on serial port
3. Check controller config: `grace_controllers.yaml` parameters
4. Debug:
   ```bash
   ros2 control list_controllers
   ros2 control list_hardware_interfaces
   ```

### Robot Drifts/Odometry Incorrect
**Symptom:** Robot doesn't drive straight, odometry way off

**Solutions:**
1. Recalibrate wheel parameters (radius, separation)
2. Check encoder wiring and pulse counts
3. Ensure motors have similar performance
4. Test each motor individually
5. Verify encoder resolution in firmware

### IMU Not Detected
**Symptom:** `OSError: [Errno 121] Remote I/O error`

**Solutions:**
1. Check I2C wiring (swap SDA/SCL if needed)
2. Verify 3.3V power (not 5V!)
3. Test I2C: `i2cdetect -y 1`
4. Check I2C is enabled in device tree
5. Reduce I2C clock speed if needed:
   ```bash
   # Edit /boot/firmware/config.txt
   dtparam=i2c_arm=on,i2c_arm_baudrate=100000
   ```

---

## 📊 Performance Monitoring

### System Resources
```bash
# Check CPU usage
htop

# Monitor ROS 2 node performance
ros2 topic hz /scan
ros2 topic hz /grace_controller/odom
ros2 topic bw /scan  # Bandwidth

# Check TF latency
ros2 run tf2_ros tf2_monitor odom base_footprint
```

### Real-time Priority (for better performance)
```bash
# Set higher priority for critical nodes
sudo chrt -f 50 ros2 run rplidar_ros rplidar_node --ros-args --params-file ...
```

---

## 🎓 Next Steps

1. ✅ Complete hardware setup and testing
2. ✅ Calibrate odometry and validate accuracy
3. ✅ Create maps of your environment
4. ✅ Tune Nav2 parameters for your robot's dynamics
5. ⏩ Implement recovery behaviors
6. ⏩ Add safety features (emergency stop, obstacle avoidance)
7. ⏩ Deploy autonomous missions

---

## 📞 Support Resources

- **ROS 2 Humble Docs:** https://docs.ros.org/en/humble/
- **Nav2 Documentation:** https://navigation.ros.org/
- **RPLidar Driver:** https://github.com/Slamtec/rplidar_ros
- **ros2_control Guide:** https://control.ros.org/

---

**Happy Robotics! 🤖**
