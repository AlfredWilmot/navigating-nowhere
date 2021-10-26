# Navigating Nowhere

This set of instructions aims to outline how to setup the ROS navigation stack on a simulated robot in ROS Melodic (on Ubuntu 18.04). 

Setting-up the Boston Dynamics *spot* robot using the [clearpath robotics ROS package](https://github.com/clearpathrobotics/spot_ros) for the hardware driver, and the [*CHAMP* ROS package](https://github.com/chvmp/champ) for simulating the robot in Gazebo and configuration files for the navigation stack.

1) Follow the README of the [*CHAMP* ROS package](https://github.com/chvmp/champ). Validate installation by running the SLAM demo.

2) Experimenting in simulation, need to tune costmap and control params for more efficient navigation.
   - `roslaunch champ_config gazebo.launch`
   - `roslaunch champ_config gmapping.launch rviz:=true`

3) Installed the [spot config folder](https://github.com/chvmp/robots/tree/master/configs/spot_config) by following the instructions on the [*CHAMP* robots repo](https://github.com/chvmp/robots.git). Running SLAM in simulation with spot is set-up as follows:
   - `roslaunch spot_config gazebo.launch`
   - `roslaunch spot_config slam.launch rviz:=true`

---
## Notes

Clear costmap: 
- `rosservice call /move_base/clear_costmaps`

Reset model poses in Gazebo:
- `Ctrl+Shift+R`

ROS Nav Tutorials:
- [ROS Navigation Tuning Guide](https://kaiyuzheng.me/documents/navguide.pdf)
- [Advanced ROS Navigation configuration notes](https://blog.zhaw.ch/icclab/configuring-the-ros-navigation-stack-on-a-new-robot/)

Building CHAMP package on a new computer:
1) I am sure there is a "proper" way of dealing with this using git submodules, but I can't figure it out! 
2) Navigate to the include directory of the champ metapackage, clone the champlib repo, then rename it to "champ"
- `roscd champ/include`
- `git clone --recursive https://github.com/chvmp/libchamp.git`
- `mv libchamp/ champ/`
3) Assuming all the rosdeps are installed, should be able to build the workspace using `catkin_make`.

---