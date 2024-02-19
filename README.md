# Sanitizer-Robot-ROS2

This project was developed for the final examination of the course Autonomous and Mobile Robotics for Master's degree in Automation Engineering at the University of Bologna.

The objective of this project was to implement a Sanitizer Robot to autonomously map, localize, navigate, and disinfect unknown environments with UV lamps for corona virus inactivation.

The project included 4 stages as depicted below: 

![image](https://github.com/saa-97/Sanitizer-Robot-ROS2/assets/145654679/96f2712f-1372-42a5-990d-92fa0dca3a96)

The project utilizes the ROS2 package [explore_lite](https://github.com/robo-friends/m-explore-ros2) for autonomous mapping of the environment.

## Requirements
The package requires the following to run successfully:
1. ROS2 Humble
2. Big House simulation Environment
The package was developed on Ubuntu 22.04 

## Building the package
In order to build the package, put the **explore** and **amr_project** folders into your ROS2 workspace and build using the following commands:
```
colcon build --packages-select explore_lite
colcon build --packages-select amr_project  
```
And then install by using:
```
. install/setup.bash
```

## Running the tasks
The Big House simulation environment can be launched by using:
```
ros2 launch turtlebot3_gazebo turtlebot3_bighouse.launch.py
```

The autonmous mapping can be done by running the following commands in three terminals:
1. Launch Gazebo in the first terminals:
   ```
   ros2 launch turtlebot3_gazebo turtlebot3_bighouse.launch.py
   ```
2. Launch RVIZ2 in order to visualize the mapping the process and to launch the NAV2 stack:
   ```
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True params_file:=/$HOME/dev_ws/src/amr_project/config/nav2_params.yaml slam:=True
   ```
   Make sure that the **params_file** argument contains the correct path to the parameters file while launching.
