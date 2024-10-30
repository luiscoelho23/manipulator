# Manipulator

A FRANKA EMIKA Panda robot. </br>
DMP generator.  </br>
RL Envoriments. </br>

## DEPS
- cpp-utils
- mplearn
- mplibrary
- CMA-ES
- tinyxml2

### Vizualization
- ros2-gazebo
- rviz2

## Build

```
cd ws
colcon build
. install/setup.bash <-- source .bashrc
```

## Run 
#### Tested on: 
- WSL2 - Ubuntu 22.04.03 LTS </br>
- Ros2 - Humble

Erase manipulator.urdf from resources/robot_description and generate new with xacro manipulator.urdf.xacro

To launch rviz vizualization:
```
ros2 launch manipulator rviz.launch.py
```

To launch the simulation:
```
ros2 launch manipulator simulation.launch.py
```
--symlink-install