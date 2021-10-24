# Navigating Nowhere

This set of instructions aims to outline how to setup the ROS navigation stack on a simulated robot in ROS Melodic (on Ubuntu 18.04). First I will document my experience following this set of tutorials from Addison Sears-Collins:

- [Setting up the ROS navigation stack for a simulated robot](https://automaticaddison.com/setting-up-the-ros-navigation-stack-for-a-simulated-robot/)

---
### Making a Moving Base

For this section, I followed the *[How to build a Simulated Mobile Robot Base Using ROS](https://automaticaddison.com/how-to-build-a-simulated-mobile-robot-base-using-ros/)* tutorial from Addison Sears Collins.

---

1) Create a catkin_workspace.

2) Install prerequisite packages:
     - `sudo apt-get install ros-melodic-ros-control`
     - `sudo apt-get install ros-melodic-ros-controllers`
     - `sudo apt-get install ros-melodic-gazebo-ros-control` 

3) Create a rospkg in the catkin workspace, and build the workspace:
     - `catkin_create_pkg mobile_manipulator_body std_msgs roscpp rospy`
     - `catkin_make --only-pkg-with-deps mobile_manipulator_body`

4) Create these folders in the context of the created "mobile_manipulator_body" rospkg:
      - `mkdir config launch meshes urdf`

5) Download the meshes from [this](https://drive.google.com/drive/folders/1-NTb0bWy1NfOFHB96pBzhM4gDQA02zE-?usp=sharing) link, and place them in the "meshes" folder.

6) Download the "robot_base.urdf" from [here](https://drive.google.com/drive/folders/1oX9Eyd1fKWX1HOQfMhK5aJoHKeJjznzM?usp=sharing) and place it in the urdf folder.

7) Navigate to the urdf folder, and use rviz to inspect the model:
   - `roscd mobile_manipulator_body/urdf/`
   - `roslaunch urdf_tutorial display.launch model:=robot_base.urdf`

8) Looks ok? Great! Download the "control.yaml" configuration file from [here](https://drive.google.com/drive/folders/1UlkrsKflhNXCpGrL4QA26yL3ezgKfteg?usp=sharing) and place it in the config folder.

9) Download the gazebo launch file from [here](https://drive.google.com/drive/folders/1Acv58Up41u5pYDM5yE1jru_YoVbn_rCl?usp=sharing), place it in the launch folder, then run the simulation:
   - `roslaunch mobile_manipulator_body base_gazebo_control.launch`

10) Make a note of the full context of the "name_space/cmd_vel" topic:
    - `rostopic list | grep cmd_vel`

11) The simulated robot can be driven by publishing to the "name_space/cmd_vel" topic. Open-up the rqt robot steering gui and paste the full context of the "name_space/cmd_vel" topic (in this example it's "/robot_base_velocity_controller/cmd_vel"):
    - `rosrun rqt_robot_steering rqt_robot_steering`

---
### Making a Robot Arm
For this section, I followed the *[How to build a Simulated Robot Arm using ROS](https://automaticaddison.com/how-to-build-a-simulated-robot-arm-using-ros/)* tutorial from Addison Sears Collins.

---

1) Download and place the ["robot_arm.urdf"](https://drive.google.com/drive/folders/1oX9Eyd1fKWX1HOQfMhK5aJoHKeJjznzM?usp=sharing) into the urdf folder.

2) Following the tutorial, I got an error rgarding the joint-stat-publisher-gui when running "roslaunch urdf_tutorial display.launch model:=robot_arm.urdf", so I installed the package as requested:
   - `sudo apt-get install ros-melodic-joint-state-publisher-gui`

3) Download then place the ["arm_control.yaml"](https://drive.google.com/drive/folders/1UlkrsKflhNXCpGrL4QA26yL3ezgKfteg?usp=sharing) and the ["join_state_controller.yaml"](https://drive.google.com/drive/folders/1UlkrsKflhNXCpGrL4QA26yL3ezgKfteg?usp=sharing) files into the config folder.

4) Download and place the ["arm_gazebo_control.launch"](https://drive.google.com/drive/folders/1Acv58Up41u5pYDM5yE1jru_YoVbn_rCl?usp=sharing) file into the launch folder.

5) when running the gazebo simulation, the simulated arm can be controlled via the "/arm_controller/command" topic:
   - `roslaunch mobile_manipulator_body arm_gazebo_control.launch`
   - `rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["arm_base_joint","shoulder_joint", "bottom_wrist_joint", "elbow_joint","top_wrist_joint"], points: [{positions: [0, 0, 0, 0, 0], time_from_start: [1,0]}]}' -1`


---
### Making a mobile manipulator
 For this section, I followed the [_How to build a simulated mobile manipulator in ROS_](https://automaticaddison.com/how-to-build-a-simulated-mobile-manipulator-using-ros/) tutorial from Addison Sears Collins.

---
This section is essentially the same as the previous two sections, except with different urdf/ config files.

---
### Navigating a mobile manipulator
For this section, I followed the [_Setting Up the ROS Navigation Stack for a Simulated Robot_](https://automaticaddison.com/setting-up-the-ros-navigation-stack-for-a-simulated-robot/) tutorial from Addison Sears Collins.

---

The ROS Navigation Stack consists of a set of packages that enable the safe movement of a robot from an initial location to a goal location.


1) Create 
