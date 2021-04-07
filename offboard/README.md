# Offboard Control
* [px4 User Guide](https://docs.px4.io/master/en/simulation/ros_interface.html)

## Launching Gazebo with ROS Wrappers
```
cd <PX4-Autopilot_clone>
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
```

## Run the Control Node
```
cd <your workspace>
catkin_make
source devel/setup.bash
```

* For position control
```
rosrun offboard pos_ctrl_node
```

* For velocity control
```
rosrun offboard vel_ctrl_node
```

