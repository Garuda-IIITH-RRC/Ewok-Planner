# Ewok-Planner
Download QGC to Control the Drone

# How To Run Simulation
In terminal 1 start px4 Autopilot
### Terminal 1:-
```bash
cd PX4-Autopilot/
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch px4 mavros_posix_sitl.launch
```

In same terminal type
```bash
commander takeoff
```


### Terminal 2:-
```bash
cd Ewok_ws/
source devel/setup.bash
roslaunch ewok_simulation trajectory_replanning_rishabh.launch
```




In QGC
change Flight mode to offboard mode
