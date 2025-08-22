# ROS2 LiDAR Object Follower

This project demonstrates LiDAR based object following in a differential drive robot using Gazebo and ROS 2. Package will search for the nearest obstacle detected in LaserScan readings and make the robot follow it. Check out the complete project simulation [here](https://youtu.be/btLDHNtah-4).

## System Requirements

- **Ubuntu**: 24.04 LTS
- **ROS 2**: Jazzy Jalapa
- **Gazebo**: Harmonic

## Usage

### Prerequisites

Install required ROS 2 packages:

```bash
sudo apt install ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher \
                 ros-jazzy-xacro \
                 ros-jazzy-teleop-twist-keyboard \
                 ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge \
                 ros-jazzy-sensor-msgs \
                 ros-jazzy-geometry-msgs
```

### Installation & Setup

1. **Create Workspace and Clone Repository**

```bash
mkdir -p my_ws/src && cd my_ws/src
git clone https://github.com/yashikasharma0301/lidar_object_follower.git
```

2. **Build the Workspace**

```bash
cd ..
colcon build
```

3. **Source the Workspace**

```bash
source install/setup.bash
```

### Running the Simulation

1. **Launch Robot with LiDAR and Target Object in Gazebo**

```bash
ros2 launch lidar_object_follower object_follower.launch.py
```
<img width="1854" height="1168" alt="image" src="https://github.com/user-attachments/assets/d6909335-a58e-4b75-9db4-4223c28e062d" />


To enable LiDAR visualization:
- Click on the three dots on the upper right corner of your Gazebo window
- Search for **Visualize Lidar** from the menu and click on it
- Refresh the lists of topics and choose the '/scan' topic. Ensure that **Display Lidar Visualisation** is checked.

2. **Start the LIDAR Object Follower Node**

In a new terminal, source the workspace and run the follower node:

```bash
cd my_ws
source install/setup.bash
ros2 run lidar_object_follower follower_node
```

Use the Gazebo interface to move the object around. 
<img width="1854" height="1168" alt="image" src="https://github.com/user-attachments/assets/94e593f4-651f-4e36-94ae-ff914da374f5" />


The robot will follow the object as long as it is in the LiDAR detection range.

## Credits & References

**Robot Model**: This project uses a URDF model adapted from the TortoiseBot example in the [OSRF ROS Book](https://github.com/osrf/rosbook/blob/master/code/tortoisebot/tortoisebot.urdf). The original model has been modified for ROS 2 Jazzy and Gazebo Harmonic integration with LiDAR sensor capabilities.

**Original Authors**: Open Source Robotics Foundation (OSRF)
