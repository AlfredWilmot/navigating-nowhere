# Navigating Nowhere

This set of instructions aims to outline how to setup the ROS navigation stack on a simulated robot in ROS Melodic (on Ubuntu 18.04). 

These notes are based on instructions from [this guide](https://www.clearpathrobotics.com/assets/guides/kinetic/ros/ROS%20Navigation%20Basics.html) from clearpath robotics for their jackal robot.

- Once everything shown in the guide has been dowloaded, make sure all the rospackage dependencies are met:

  - `rosdep install --from-paths src --ignore-src -r -y`

The various components of interest form the guide are as follows:
- Simulation
  - `roslaunch jackal_gazebo jackal_world.launch config:=front_laser`
- Navigation paramters (move_base)
  - `roslaunch jackal_navigation odom_navigation_demo.launch`
- Map-building (gmapping)
  - `roslaunch jackal_navigation gmapping.launch`
- Data visualization 
  - `roslaunch jackal_viz view_robot.launch config:=gmapping`
