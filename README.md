# Manipulator

A simulation in gazebo with rviz vizualization of a FRANKA EMIKA Panda robot. </br>


## Build

```
cd colcon_ws
colcon build --packages-select manipulator --symlink-install
. install/setup.bash
```

#### Build DMP Loader deps
```
colcon build --packages-select cpp_utils mplibrary --ament-cmake-args -DCMAKE_CXX_COMPILER=g++-11
```

## Run 
#### Tested on: 
- Ubuntu 22.04.04 LTS </br>
- Kernel Vesion 6.5.0-27-generic </br>
- Ros2 - Humble

To launch rviz vizualization:
```
ros2 launch manipulator inRviz.launch.py
```

To launch the simulation:
```
ros2 launch manipulator simulation.launch.py
```
To run the controller, In another terminal, run the following:</br>
##### Make python script executable first !!


```
ros2 run manipulator controller.py <ang1> <ang2> <ang3> <ang4> <ang5> <ang6> <ang7>
```
