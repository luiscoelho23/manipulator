# Manipulator

A simulation in gazebo with rviz vizualization of a FRANKA EMIKA Panda robot. </br>


## Build

```
colcon build --packages-select manipulator --symlink-install
. install/setup.bash
```

## Run 
#### Tested on: 
- Ubuntu 22.04.04 LTS </br>
- Kernel Vesion 6.5.0-27-generic </br>
- Ros2 - Humble

To launch the simulation:
```
ros2 launch manipulator simulation.launch.py
```
To run the controller, In another terminal, from inside the workspace run the following:</br>
##### Make python script executable first !!


```
. install/setup.bash
ros2 run manipulator controller.py
```
