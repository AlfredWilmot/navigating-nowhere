# Navigating Nowhere

This set of instructions aims to outline how to setup the ROS navigation stack on a simulated robot in ROS Melodic (on Ubuntu 18.04). 

Setting-up the Boston Dynamics *spot* robot using the [clearpath robotics ROS package](https://github.com/clearpathrobotics/spot_ros) for the hardware driver, and the [*CHAMP* ROS package](https://github.com/chvmp/champ) for simulating the robot in Gazebo and configuration files for the navigation stack.

1) Follow the README of the [*CHAMP* ROS package](https://github.com/chvmp/champ). Validate installation by running the SLAM demo.

2) Experimenting in simulation, need to tune costmap and control params for more efficient navigation.
   - `roslaunch champ_config gazebo.launch`
   - `roslaunch champ_config gmapping.launch rviz:=true`