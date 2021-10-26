# Navigating Nowhere

This set of instructions aims to outline how to setup the ROS navigation stack on a simulated robot in ROS Melodic (on Ubuntu 18.04). 

1) Setting-up the catkin workspace:
   - Following [this](https://towardsdatascience.com/ros-autonomous-slam-using-randomly-exploring-random-tree-rrt-37186f6e3568) guide to test Autonomous SLAM using the *Rapidly Exploring Random Tree (RRT) Algorithm*.

   - [ros_autonomous_slam](https://github.com/fazildgr8/ros_autonomous_slam.git) repo.

   - [Nav tuning guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#tuning-guide) from turtlebot docs.

   - Having issues with this guy's implementation, try the [*rrt_exploration* ros wiki](http://wiki.ros.org/rrt_exploration/Tutorials/singleRobot) instead.

---
   ### Notes
- If anything ROS related is being a pain in the arse, try the `killall` command to get rid of any loose ends (e.g. `killall gzserver`).
---