# Navigating Nowhere

This set of instructions aims to outline how to setup the ROS navigation stack on a simulated robot in ROS Melodic (on Ubuntu 18.04). 

First I will document my experience following this set of tutorials from Addison Sears-Collins.

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

1) Simply set-up the packages as is described by the tutorial; ensure that the path to the gazebo environement is set-up such that it will launch the "postworld" map.

2) ROS Nav parameters to enable autonomous navigation: Costmaps.
   - Costmaps are used to store information about the environment
     - *Global Costmap*: used to calculate the shortes path from a start point to an end point.
     - *Local Costmap*: Use for obstacle avoidance.

3) Setup the various config files.
   - Costmap parameter files:
     - [Common](https://drive.google.com/file/d/1uBClSpjCtTPbDtAD-Ae13RcL40cx16eg/view?usp=sharing)
     - [Global](https://drive.google.com/file/d/1kv2ZKEFp2YUHZ7K_TjOKDYSFzBnNuPmW/view?usp=sharing)
     - [Local](https://drive.google.com/file/d/1L1mO2myoFKJtZhCnDSeWHwnLMzH3Y9Cg/view?usp=sharing)
   - [Base Local Planner](https://drive.google.com/file/d/1_2LhNDF9nPPzDSN-RKUuhPuYMZXGYmZS/view?usp=sharing) (used to compute the velocity commands sent to the robot base controller).
   - [RViz settings](https://drive.google.com/file/d/1HO_udwH6aVXy1Ym209lBYSC7msYtIrOG/view?usp=sharing)

4) Ensure the ROS navigation stack is installed (confirm by checking for the existence of the amcl rospackage):
   - `sudo apt-get install ros-melodic-navigation`
   - `rospack find amcl`

5) Creating a map using the *ROS Hector-SLAM* Package by first installing *QT4* then download the Hector-SLAM package.
   - Install *QT4*:
     - `sudo add-apt-repository ppa:rock-core/qt4`
     - `sudo apt update`
     - `sudo apt-get install qt4-qmake qt4-dev-tools`
   - Clone *Hecotr-SLAM* pkg into src folder:
     - `git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git`
   - Ensure the coordinate frame parameters in "/hector_slam/hector_mapping/launch/mapping_default.launch" are set-up with the right names & options.

6) Setup the simulation and launch the mapping algorithm; this will generate a map that can be saved as a png and subsequently loaded onto a map_server for subsequent use (requires installation of the map-server pkg). The robot needs to move quite slowly around the environment in order to generate an accurate map-- if it moves too fast the map will be way off.
    - `roslaunch mobile_manipulator mobile_manipulator_gazebo.launch`
    - `roslaunch hector_slam_launch tutorial.launch`
    - `rosrun rqt_robot_steering rqt_robot_steering`

7) Once the robot has been made to navigate around its environment, and a map can bee seen in RViz, store the map in a "maps" folder:
   - `roscd mobile_manipulator/maps`
   - `rosrun map_server map_saver -f test`

8) To load a saved map, set-up a roscore, run the map server and load the "test.yaml" map you made to the rosparam server. Now the map can be inspected in rviz by interrogating the "/map" topic via the Map display plugin.
   - `roscore`
   - `rosrun map_server map_server test.yaml`
   - `rviz` (then within rviz: "Add" -> "Map" -> select topic "/map")