# Manipulator

A FRANKA EMIKA Panda robot. </br>
DMP generator.  </br>
Trajectory Mapping. <br>
RL Envoriments. </br>

## DEPS
- cpp-utils
- mplearn
- mplibrary
- CMA-ES
- tinyxml2
- ros2-controller-manager

### Vizualization & Simulation
- ros2-gazebo
- rviz2

## Build

```
cd ws
colcon build
source install/setup.bash
```

*source install/setup.bash -> ~/.bashrc
*Erase manipulator.urdf from resources/robot_description and generate new with xacro manipulator.urdf.xacro

## Run 
#### Tested on: 
- Ubuntu 22.04.5 LTS (GNU/Linux 5.15.153.1-microsoft-standard-WSL2 x86_64) </br>
- Ros2 - Humble

To launch rviz vizualization:
```
ros2 launch manipulator rviz.launch.py
```

To launch the simulation:
```
ros2 launch manipulator simulation.launch.py
```

To run DMP + RL Model
```
ros2 run manipulator load_dmps.py
```

To Generate DMP and Mapping trajectories to Franka Panda
```
cd manipulator
python3 dmp_generator.py 
python3 trajectory_mapping.py
```

To train Rl model
```
cd manipulator/rl
python3 train.py
```