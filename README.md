# Lab 1 Exercise: diffdrive_bot Implementation

**Course:** Introduction to Robotics and Intelligent Systems  
**Academic Year:** 2026  
**Student Workspace:** `~/ros2_ws/src/lab1_exercise`

---

## Overview

This repository contains the complete implementation for Lab 1, extending the `diffdrive_bot` mobile robot in Gazebo Sim (Harmonic / ROS 2) with custom sensors, ROS 2 topics, parameters, custom message/service/action interfaces, and autonomous behavior nodes.

The workspace contains four packages:
1. `diffdrive_bot`: Robot description (URDF/Xacro), Gazebo simulation worlds, bridge configurations, and base controllers.
2. `lab_interfaces`: Provided message, service, and action interfaces (`ObstacleInfo`, `SetSpeed`, `DriveDistance`).
3. `my_robot_interfaces`: Custom interfaces created for this lab (`RobotStatus.msg`, `SetTurnSpeed.srv`, `Rotate.action`).
4. `my_robot_nodes`: Custom nodes (`turn_away`, `rotate_server`, `square_driver`) and launch files (`square.launch.py`).

---

## Building the Packages

From the workspace root (`~/ros2_ws`):

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select diffdrive_bot lab_interfaces my_robot_interfaces my_robot_nodes
source install/setup.bash
```

To run the unit tests and linters:

```bash
colcon test --packages-select my_robot_interfaces my_robot_nodes
colcon test-result --verbose
```

---

## Step-by-Step Instructions

### Step 1: Depth Camera and IMU Integration
- **URDF (`diffdrive_bot/urdf/diffdrive_bot.urdf.xacro`)**:
  - Added `depth_camera_link` on the rear face of the chassis pointing backwards ($\pi$ yaw offset), publishing on `/depth_camera` at 10 Hz (`R_FLOAT32`, 320x240, 0.1–10 m range).
  - Added `imu_link` at the chassis center, publishing on `/imu` at 50 Hz.
- **World (`diffdrive_bot/worlds/obstacles.sdf`)**: Loaded `gz-sim-imu-system` (`gz::sim::systems::Imu`).
- **Bridge (`diffdrive_bot/config/bridge.yaml`)**: Added bridges for `/depth_camera/image`, `/depth_camera/points`, `/depth_camera/camera_info`, and `/imu`.
- **Run & Verify**:
  ```bash
  ros2 launch diffdrive_bot display.launch.py
  ros2 launch diffdrive_bot gazebo.launch.py
  ros2 topic hz /imu
  ros2 topic echo /imu --once
  ```

### Step 2 & 3: Reactive Obstacle Avoidance Node with Parameters
- **Node (`my_robot_nodes/turn_away.py`)**:
  - Subscribes to `/obstacle` (`lab_interfaces/msg/ObstacleInfo`) and publishes to `/cmd_vel` (`geometry_msgs/msg/Twist`).
  - Drives forward when `too_close` is `false`; turns away when `too_close` is `true`.
  - Dynamic parameters: `forward_speed` (default `0.2` m/s) and `turn_speed` (default `0.5` rad/s).
- **Run & Verify**:
  ```bash
  ros2 launch diffdrive_bot control.launch.py
  ros2 run my_robot_nodes turn_away
  ros2 param set /turn_away forward_speed 0.4
  ```

### Step 4: Custom Message `RobotStatus.msg`
- **Interface (`my_robot_interfaces/msg/RobotStatus.msg`)**:
  - `string state` (`"forward"`, `"turning"`, `"stopped"`)
  - `float32 obstacle_distance`
- Published on `/robot_status` with each obstacle update.
- **Run & Verify**:
  ```bash
  ros2 topic echo /robot_status
  ```

### Step 5: Service Interfaces
- **Services in `turn_away`**:
  - `/turn_away_enable` (`std_srvs/srv/SetBool`): Enables or disables autonomous driving (initial state is disabled/`stopped`).
  - `/set_turn_speed` (`my_robot_interfaces/srv/SetTurnSpeed`): Sets `turn_speed` parameter within bounds `(0, max_turn_speed]`.
- **Run & Verify**:
  ```bash
  ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: true}"
  ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 0.8}"
  ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: false}"
  ```

### Step 6: Rotation Action Server
- **Interface (`my_robot_interfaces/action/Rotate.action`)**:
  - Goal: `float32 angle` (radians, positive turns left).
  - Result: `float32 turned`.
  - Feedback: `float32 turned`.
- **Node (`my_robot_nodes/rotate_server.py`)**:
  - Tracks accumulated yaw angle from `/odom` to handle $\pm\pi$ wrap-around correctly.
  - Multi-threaded executor for non-blocking feedback and goal handling.
- **Run & Verify**:
  ```bash
  ros2 run my_robot_nodes rotate_server
  ros2 action send_goal --feedback /rotate my_robot_interfaces/action/Rotate "{angle: 1.5708}"
  ```

### Step 7: Square Trajectory Driver and Launch File
- **Node (`my_robot_nodes/square_driver.py`)**:
  - Coordinates `/drive_distance` (`lab_interfaces/action/DriveDistance`) and `/rotate` (`my_robot_interfaces/action/Rotate`) to drive a 4-sided square of configurable `side_length`.
- **Launch File (`my_robot_nodes/launch/square.launch.py`)**:
  - Launches `control.launch.py`, `rotate_server`, and `square_driver` with parameter `side_length`.
- **Run & Verify**:
  ```bash
  ros2 launch my_robot_nodes square.launch.py side_length:=0.5
  ```

---

## Troubleshooting & What Did Not Work (and Why)

### Issue 1: Task 7 Goal Rejection on Initial Startup
- **Symptom**: When running `ros2 launch my_robot_nodes square.launch.py side_length:=0.5`, `square_driver` discovered the action servers, but `/drive_distance` immediately rejected the 0.50 m goal.
- **Root Cause**: `drive_distance_server.py` in `diffdrive_bot` contains the validation check:
  ```python
  if goal.distance <= 0.0 or self.position is None:
      return GoalResponse.REJECT
  ```
  `self.position` is only populated once the first `/odom` message is received from Gazebo. While `square_driver`'s `wait_for_server()` succeeded as soon as the ROS 2 action graph discovered the node, Gazebo takes 15–25 seconds to start up, load plugins, and spawn the robot model (`SPAWN_DELAY_S = 2.0`). Because odometry was not yet streaming, `self.position` was `None` when the goal arrived, leading to immediate rejection.
- **Fix**:
  1. Subscribed `square_driver` to `/odom`.
  2. In `SquareDriver.wait_for_servers()`, added an explicit check waiting for `self.odom_received` (with a 90-second timeout to allow Gazebo to fully load).
  3. Added goal rejection retry logic with exponential backoff in `drive_distance()` and `rotate()`.
  4. Added a `stop()` method to safely halt robot motion upon completion.
