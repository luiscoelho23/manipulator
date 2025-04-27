# Manipulator

A ROS2 package for the FRANKA EMIKA Panda robot manipulator.

![Franka Panda Robot](https://franka.de/wp-content/uploads/2020/07/panda-robot-arm-1.jpg)

## Overview

This package provides a complete setup for working with the FRANKA EMIKA Panda robot, including:

- Collision detection environment
- Robot simulation in Gazebo
- Controller configuration
- Python utilities for kinematics and robot control

## Repository Structure

```
manipulator/
├── CMakeLists.txt          # CMake build file
├── package.xml             # ROS2 package metadata
├── include/                # C++ header files
├── src/                    # C++ source files
│   ├── collision_detection_env.cpp
│   └── run_cd.cpp
├── manipulator/            # Python modules
│   ├── __init__.py
│   ├── controller.py
│   └── kdl_parser.py
├── launch/                 # Launch files
│   └── launch.py
└── resources/              # Resource files
    ├── robot_description/  # URDF/XACRO files
    ├── controllers/        # Controller configurations
    ├── worlds/             # Gazebo world files
    └── rviz/               # RViz configurations
```

## Dependencies

### Core Dependencies
- ROS2 Humble
- rclcpp / rclpy
- ros2_control
- ros2_controllers
- gazebo-ros2-control
- Fast Collision Detection Library (FCL)
- libccd
- Assimp
- URDF

### Visualization & Simulation
- Gazebo
- RViz2

## Installation

1. Make sure you have ROS2 Humble installed. If not, follow the [official ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html).

2. Install required dependencies:
   ```bash
   sudo apt update
   sudo apt install -y \
     ros-humble-ros2-control \
     ros-humble-ros2-controllers \
     ros-humble-gazebo-ros2-control \
     ros-humble-xacro \
     ros-humble-joint-state-publisher-gui \
     libassimp-dev \
     libccd-dev \
     libfcl-dev
   ```

3. Clone the repository into your workspace:
   ```bash
   mkdir -p ~/ws_manipulator/src
   cd ~/ws_manipulator/src
   git clone https://github.com/lsuiceolho23/manipulator.git
   ```

4. Build the workspace:
   ```bash
   cd ~/ws_manipulator
   colcon build --symlink-install
   source install/setup.bash
   ```

5. For convenience, add the setup to your `.bashrc`:
   ```bash
   echo "source ~/ws_manipulator/install/setup.bash" >> ~/.bashrc
   ```

## Usage

### Launch the Simulation

The package includes a comprehensive launch file with several configurable options:

```bash
ros2 launch manipulator launch.py
```

Launch arguments:
- `controller_type`: Choose between `forward_position_controller` or `joint_trajectory_controller` (default: forward_position_controller)
- `rviz`: Enable/disable RViz visualization (default: true)
- `joint_state_publisher`: Enable/disable joint state publisher GUI (default: true)
- `gazebo`: Enable/disable Gazebo simulation (default: true)
- `world`: World file to load in Gazebo (default: world.xml)

Examples:
```bash
# Launch with joint trajectory controller
ros2 launch manipulator launch.py controller_type:=joint_trajectory_controller

# Launch without Gazebo (for visualization only)
ros2 launch manipulator launch.py gazebo:=false

# Launch with a custom world file
ros2 launch manipulator launch.py world:=custom_world.xml
```

### Running the Collision Detection

To run the collision detection executable:

```bash
ros2 run manipulator run_cd
```

## Development

### Robot Description

The robot description is defined using XACRO files located in the `resources/robot_description` directory. To generate the URDF file:

```bash
cd ~/ws_manipulator
ros2 run xacro xacro src/manipulator/resources/robot_description/manipulator.urdf.xacro > src/manipulator/resources/robot_description/manipulator.urdf
```

### Adding New Controllers

1. Define new controller configurations in `resources/controllers/controllers.yaml`
2. Update the launch file to use your new controller

## System Requirements

- Ubuntu 22.04 LTS or higher
- ROS2 Humble
- At least 8GB RAM recommended for Gazebo simulation

## Maintainer

- Luís Coelho (luiscoelho321@hotmail.com)

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.