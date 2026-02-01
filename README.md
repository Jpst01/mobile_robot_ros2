# Mobile Robot Simulation using ROS 2

![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?logo=ubuntu&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-22314E?logo=ros&logoColor=white)

**Repository:** https://github.com/Jpst01/mobile_robot_ros2  
**Author:** Jayeshwar P. S. Tanwar  
**Project Type:** Individual / Solo Project  
**Development Status:** Actively under development  

---

## 1. Project Overview

This repository contains a ROS 2–based mobile robot simulation developed as an individual project, focusing on low-level understanding of robot description, TF frames, sensor integration, and simulation behavior. The project uses ROS 2 Jazzy Jalisco with Gazebo Harmonic and RViz2 to simulate a differential-drive mobile robot equipped with common sensors.

---

## 2. Motivation and Learning Objectives

The primary goals of this project are:

- To gain a solid understanding of ROS 2 system architecture and middleware behavior.
- To design and simulate a mobile robot with a clean and correct TF tree.
- To integrate multiple sensors and validate their data in simulation.
- To prepare a strong foundation for autonomous navigation using Nav2.

This project is used as a learning and validation platform for robotics fundamentals relevant to real-world embedded and robotic systems.

---

## 3. System Architecture

The system follows a standard ROS 2 architecture:

- Robot description published via `robot_state_publisher`
- Simulation handled by Gazebo Harmonic
- Sensor data published through Gazebo ROS plugins
- Visualization and debugging performed in RViz2
- TF tree connecting `map`, `odom`, and robot frames
- Motion commands published to the robot controller in simulation

All components are integrated using ROS 2 launch files and built using `colcon`.

---

## 4. Robot Design and Sensor Configuration

### Mechanical Structure

- Mobile robot with a rigid chassis composed of primitive geometries:
  - **Box**: main body
  - **Cylinders**: left and right wheels
  - **Sphere**: support / caster element
- Differential-drive configuration

### Sensors

- **IMU**
  - Mounted on the front of the robot
  - Publishes orientation and angular velocity data
- **RGB-D Camera**
  - Mounted on the front of the robot
  - Provides color and depth streams
- **2D LiDAR**
  - Mounted on the top of the robot
  - Used for mapping and localization
  
  ---

## 5. Software Stack and Versions

- **Operating System:** Ubuntu 24.04
- **ROS 2:** Jazzy Jalisco
- **Simulator:** Gazebo Harmonic
- **Visualization:** RViz2
- **Programming Languages:** C++, Python
- **Robot Description:** URDF with XACRO
- **Build System:** colcon

---

## 6. Package Structure


├── mobile_description  
│   ├── CMakeLists.txt  
│   ├── config  
│   │   └── mobile_robot  
│   │       ├── ros2_controllers_template.yaml  
│   │       └── ros2_controllers.yaml  
│   ├── launch  
│   │   └── robot_state_publisher.launch.py  
│   ├── LICENSE  
│   ├── package.xml  
│   ├── rviz  
│   │   └── mobile_robot_description.rviz  
│   └── urdf  
│       ├── control  
│       │   ├── gazebo_sim_ros2_control.urdf.xacro  
│       │   └── mobile_robot_ros2_control.urdf.xacro  
│       ├── mech  
│       │   └── mobile_robot_base.urdf.xacro  
│       ├── robots  
│       │   └── mobile_robot.urdf.xacro  
│       └── sensors  
│           ├── imu.urdf.xacro  
│           ├── lidar.urdf.xacro  
│           └── rgbd_camera.urdf.xacro  
├── mobile_robot  
│   ├── CMakeLists.txt  
│   ├── LICENSE  
│   └── package.xml  
├── mobile_robot_bringup  
│   ├── CMakeLists.txt  
│   ├── launch  
│   │   ├── load_ros2_controllers.launch.py  
│   │   └── mobile_robot_navigation.launch.py  
│   ├── LICENSE  
│   └── package.xml  
├── mobile_robot_gazebo  
│   ├── CMakeLists.txt  
│   ├── config  
│   │   └── ros_gz_bridge.yaml  
│   ├── launch  
│   │   └── mobile_robot.gazebo.launch.py  
│   ├── LICENSE  
│   ├── models  
│   │   └── depot  
│   │       ├── materials  
│   │       ├── meshes  
│   │       ├── model.config  
│   │       └── model.sdf  
│   ├── package.xml  
│   ├── rviz  
│   │   └── mobile_robot_gazebo_sim.rviz  
│   └── worlds  
│       ├── depot.world  
│       └── empty.world  
├── mobile_robot_localization  
│   ├── CMakeLists.txt  
│   ├── config  
│   │   └── ekf.yaml  
│   ├── launch  
│   │   └── ekf_gazebo.launch.py  
│   ├── LICENSE  
│   └── package.xml  
├── mobile_robot_navigation  
│   ├── CMakeLists.txt  
│   ├── config  
│   │   └── mobile_robot_nav2_default_params.yaml  
│   ├── LICENSE  
│   ├── maps  
│   ├── package.xml  
│   └── rviz  
│       └── nav2_default_view.rviz  
├── mobile_robot_system_tests  
│   ├── CMakeLists.txt  
│   ├── LICENSE  
│   └── package.xml  
└── README.md  



- `urdf/`: Robot model and sensor definitions
- `launch/`: Launch files for simulation and visualization
- `config/`: .yaml files
- `rviz/`: rviz configuration files

---

## 7. Simulation Details (Gazebo + RViz)

- Gazebo Harmonic used for physics simulation
- Robot spawns correctly with proper collision and visual geometry
- All sensors publish valid data streams
- RViz visualizes:
  - Robot model
  - TF tree
  - LiDAR scans
  - Camera topics
  - IMU data
- Robot motion is functional and controllable in simulation

---

## 8. SLAM and Mapping Status

### Current State

- SLAM integration is **in progress**
- Mapping is partially functional but currently unreliable

### Known Issue (Important)

- When the robot **rotates in place**, the **RViz map rotates along with the robot**
- This issue does **not** occur during pure forward or backward motion
- The resulting map becomes corrupted and unusable
- The problem is related to incorrect handling of TF frames between:
  - `map`
  - `odom`
  - `base_link`

### Transparency Note

- One recent commit message incorrectly states that this issue was solved
- That commit was **incorrect**
- The issue is **not resolved** and is under active debugging
- This README intentionally documents that mistake for clarity and honesty

---

## 9. Navigation Plan (Nav2)

Planned integration steps:

- Fix TF consistency during SLAM
- Generate a stable 2D occupancy map
- Save and reuse the generated map
- Integrate Nav2 stack
- Implement:
  - Waypoint-based navigation
  - Obstacle avoidance using the map

---

## 10. Current Limitations

- SLAM mapping is unstable during rotation
- Autonomous navigation is not yet implemented
- Focus is currently on correctness rather than performance optimization

---

## 11. Future Improvements

- Validate odometry and sensor fusion
- Add Nav2-based autonomous navigation
- Improve simulation realism (noise, friction, sensor parameters)

---

## 12. Repository Activity and Development Practices

- Regular, incremental commits
- Issues debugged step by step rather than masked
- No external templates or boilerplate used
- Commit history reflects real development progress, including mistakes and corrections

---

## 13. How to Build and Run

### Build

```bash
cd ~/mobile_robot_ws
colcon build
source install/setup.bash
ros2 launch mobile_robot_bringup mobile_robot_navigation.launch.py
