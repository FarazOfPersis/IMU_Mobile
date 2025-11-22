# 📱 Mobile IMU Data Logger & Path Estimator

A ROS2 package for collecting and processing IMU sensor data from mobile devices with real-time filtering, sensor fusion, and 3D path visualization.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Node Details](#node-details)
- [Topics](#topics)
- [Parameters](#parameters)
- [Algorithm Pipeline](#algorithm-pipeline)
- [Visualization](#visualization)
- [Troubleshooting](#troubleshooting)

---

## 🎯 Overview

This ROS2 package processes raw IMU data from mobile devices to estimate device orientation and trajectory in 3D space. It implements gyroscope bias correction, low-pass filtering, complementary filtering for orientation estimation, and double integration for position tracking with Zero Velocity Update (ZUPT) to minimize drift.

---

## ✨ Features

- **Automatic Gyroscope Bias Calibration**: Collects initial samples to calculate and remove sensor bias
- **Low-Pass Filtering**: Reduces high-frequency noise from accelerometer data
- **Complementary Filter**: Fuses accelerometer and gyroscope data for accurate orientation estimation
- **Dead Reckoning**: Double integration of acceleration to estimate velocity and position
- **Zero Velocity Updates (ZUPT)**: Reduces drift when device is stationary
- **Dynamic Constraints**: Detects flat orientation and constrains vertical drift
- **TF Broadcasting**: Publishes transform from `odom` to `phone_imu_link`
- **Path Visualization**: Publishes trajectory as `nav_msgs/Path` for RViz display

---

## 📦 Requirements

### Software
- ROS2 (Humble/Iron recommended)
- Python 3.8+
- Required Python packages:
  - `rclpy`
  - `sensor_msgs`
  - `geometry_msgs`
  - `nav_msgs`
  - `tf2_ros`
  - `tf_transformations`
  - `numpy`

### Hardware
- Mobile device with IMU sensors (accelerometer, gyroscope)
- Network connection for data transmission

---

## 🛠️ Installation
```bash
# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone or create the package
ros2 pkg create mobile_imu --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs nav_msgs tf2_ros

# Copy the node script
cd mobile_imu/mobile_imu
# Place imu_processor.py here

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --packages-select mobile_imu

# Source the workspace
source install/setup.bash
```

---

## 🚀 Usage

### 1. Configure Mobile App

- Install a sensor logger app on your mobile device (e.g., "Sensor Logger", "Physics Toolbox")
- Enable IMU sensors: accelerometer and gyroscope
- Configure data transmission to publish on topic `/mobile_imu/raw` as `sensor_msgs/Imu`
- Set appropriate sampling rate (50-100 Hz recommended)

### 2. Launch the Node

bash
# Basic launch
ros2 run mobile_imu imu_processor_node

# With custom parameters
ros2 run mobile_imu imu_processor_node --ros-args \
  -p max_bias_samples:=50 \
  -p lpf_alpha:=0.15 \
  -p filter_gain:=0.98

### 3. Visualize in RViz

bash
# Launch RViz
rviz2

# Add displays:
# - Path: Subscribe to /mobile_imu/path
# - TF: Show phone_imu_link frame
# - Set Fixed Frame to "odom"

---

## 🔧 Node Details

### Node Name
`imu_processor_node`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mobile_imu/raw` | `sensor_msgs/Imu` | Raw IMU data from mobile device |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mobile_imu/filtered` | `sensor_msgs/Imu` | Filtered IMU data with estimated orientation |
| `/mobile_imu/path` | `nav_msgs/Path` | Estimated 3D trajectory path |

### Published Transforms

| Parent Frame | Child Frame | Description |
|--------------|-------------|-------------|
| `odom` | `phone_imu_link` | Device position and orientation in odometry frame |

---

## ⚙️ Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_bias_samples` | int | 25 | Number of samples for gyroscope bias calibration |
| `lpf_alpha` | double | 0.1 | Low-pass filter coefficient (0-1, higher = more filtering) |
| `filter_gain` | double | 0.98 | Complementary filter gain (gyroscope weight) |

### Parameter Configuration Example

bash
ros2 run mobile_imu_logger imu_processor --ros-args \
  -p max_bias_samples:=50 \
  -p lpf_alpha:=0.2 \
  -p filter_gain:=0.95

---

## 🔬 Algorithm Pipeline

### Step-by-Step Processing

**1. Gyroscope Bias Calibration (Initial Phase)**
- Collects first N samples (default: 25)
- Calculates average gyroscope readings as bias
- Subtracts bias from all subsequent measurements

**2. Low-Pass Filter (Accelerometer)**
- Applies exponential moving average filter
- Formula: `filtered = α × current + (1-α) × previous`
- Reduces high-frequency noise while preserving motion

**3. Complementary Filter (Orientation Estimation)**
- Integrates gyroscope data for short-term accuracy
- Uses accelerometer for long-term stability
- Roll: `α × (roll_gyro + ω_x × dt) + (1-α) × roll_accel`
- Pitch: `α × (pitch_gyro + ω_y × dt) + (1-α) × pitch_accel`
- Yaw: Pure gyroscope integration (no correction)

**4. Gravity Compensation**
- Rotates acceleration to world frame using estimated orientation
- Subtracts gravity (9.81 m/s²) from Z-axis
- Applies noise threshold (0.08 m/s²) to filter sensor noise

**5. Dynamic Constraints**
- Detects flat orientation: `|roll| < 0.2 rad AND |pitch| < 0.2 rad`
- When flat: Forces Z acceleration, velocity, and position to zero
- Prevents vertical drift when device is on table

**6. Dead Reckoning (Position Estimation)**
- First integration: Acceleration → Velocity
- Second integration: Velocity → Position
- ZUPT: Resets velocity to zero when below threshold (0.02 m/s)

**7. Transform Broadcasting**
- Publishes TF from `odom` to `phone_imu_link`
- Enables visualization of device pose in RViz

---

## 📐 Key Equations

### Complementary Filter

For roll and pitch estimation:

$$\theta_{fused} = \alpha \cdot (\theta_{gyro} + \omega \cdot \Delta t) + (1-\alpha) \cdot \theta_{accel}$$

Where:
- $\alpha$ = 0.98 (gyroscope weight)
- $\omega$ = angular velocity (rad/s)
- $\Delta t$ = time step (s)

### Accelerometer-Based Orientation

$$roll = \arctan2(a_y, a_z)$$

$$pitch = \arctan2(-a_x, \sqrt{a_y^2 + a_z^2})$$

### Position Integration

$$v(t) = v_0 + \int a(t) \, dt$$

$$p(t) = p_0 + \int v(t) \, dt$$

---

## 🎨 Visualization

### RViz Configuration

**Display Settings:**

1. **Path Display**
   - Topic: `/mobile_imu/path`
   - Color: Choose distinct color (e.g., green)
   - Line Width: 0.02-0.05
   - Show Poses: Enable to see orientation arrows

2. **TF Display**
   - Show Frames: Enable `odom` and `phone_imu_link`
   - Show Axes: Enable to visualize device orientation
   - Frame Size: 0.1-0.3

3. **Global Options**
   - Fixed Frame: `odom`
   - Background Color: White or black for contrast
  
You could also use the rviz.config.rviz provided

### Expected Visualization

- Green line showing device trajectory
- Coordinate frame showing device orientation
- Real-time updates as device moves
- Minimal drift when stationary (ZUPT active)
  

---

## 🐛 Troubleshooting

### Issue: Vertical Drift When Stationary

**Symptoms:** Z position increases/decreases when phone is flat on table

**Solution:**
- Ensure `filter_gain` is close to 0.98
- Increase bias calibration samples (`max_bias_samples: 50`)
- Device must remain completely still during bias calibration

### Issue: Noisy Path Trajectory

**Symptoms:** Jittery path with rapid fluctuations

**Solution:**
- Increase `lpf_alpha` to 0.15-0.2 for more filtering
- Check mobile app sampling rate (reduce if too high)
- Ensure stable network connection to avoid packet loss

### Issue: No Path Displayed in RViz

**Symptoms:** Path topic exists but nothing shown

**Checklist:**
1. Verify topic: `ros2 topic echo /mobile_imu/path`
2. Check Fixed Frame in RViz is set to `odom`
3. Ensure Path display is enabled and color is visible
4. Verify node is running: `ros2 node list`

### Issue: Rapid Position Drift During Movement

**Symptoms:** Position estimate drifts far from actual location

**Explanation:** This is inherent to double integration without absolute position reference

**Mitigation:**
- Use shorter movement periods
- Reset position periodically
- Consider adding external position updates (GPS, visual odometry)
- ZUPT only helps during stationary periods

### Issue: Bias Calibration Fails

**Symptoms:** Large drift even when stationary

**Solution:**
- Place device on completely flat, stable surface during startup
- Increase `max_bias_samples` to 50-100
- Wait until node logs "Gyro Bias Calibrated" message
- Restart node if device was moved during calibration

---

## 📊 Performance Tips

### Optimal Settings for Different Scenarios

**Smooth Tracking (Low Noise):**
bash
-p lpf_alpha:=0.2 -p filter_gain:=0.98 -p max_bias_samples:=50

**Responsive Movement (Higher Bandwidth):**
bash
-p lpf_alpha:=0.05 -p filter_gain:=0.95 -p max_bias_samples:=25

**Maximum Drift Reduction:**
bash
-p lpf_alpha:=0.1 -p filter_gain:=0.99 -p max_bias_samples:=100

---


## 🙏 Acknowledgments

- Dr.Motahari and the rest of the teaching team of robotics Fall of 2025 SUT
- ROS2 Navigation Stack
- tf_transformations library
- Mobile sensor frameworks
- IMU sens community
esearch community
