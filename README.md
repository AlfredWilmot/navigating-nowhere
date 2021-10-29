# Navigating Nowhere

This set of instructions aims to outline how to setup the ROS navigation stack on a simulated robot in ROS Melodic (on Ubuntu 18.04). 

Here I am following the tutorial outlined in Ch.10 of the book [*ROS Robotics Projects*](https://www.amazon.com/ROS-Robotics-Projects-Ramkumar-Gandhinathan/dp/1838645195), authored by Ramkumar Gandhinathan & Lentin Joseph.

Overview:
- software architecture of a "typical self-driving car"
- simulating and interfacing relevant sensors in ROS/ Gazebo
- Simulation of a "Drive By Wire" (DBW) car in ROS.

The ROS packages **sensor_sim_gazebo**, **velodyne_description**, **velodyne_gazebo_plugins**, and **velodyne_simulator**, were copied directly from the "*chapter_10_ws*" section of the [git repository](https://github.com/PacktPublishing/ROS-Robotics-Projects-SecondEdition/tree/master/chapter_10_ws) associated with the [*ROS Robotics Projects*](https://www.amazon.com/ROS-Robotics-Projects-Ramkumar-Gandhinathan/dp/1838645195) book.

---
## Reading Notes

Robots are systems that can "sense/plan/act".

Popular sensors amongst self-driving cars:
- Xsens MTI IMUs
- Stereo/ monocular cameras ([Mobileye](http://www.mobileye.com/))
- Ultrasonic sensors ([Murata](http://www.murata.com/))
- LIDAR (Light Detection and Ranging): short/ medium range 
  - [Velodyne HDL-64E](https://www.mapix.com/lidar-scanner-sensors/velodyne/velodyne-hdl64/)
  - [SICK LMS 5xx/1xx](https://www.sick.com/in/en)
  - [Hokuyo LIDAR](http://www.hokuyo-aut.jp/02sensor/)
- RADAR (Radio Detection and Ranging): long range
  - [Continental ARS 30X](https://www.continental-automotive.com/getattachment/9b6de999-75d4-4786-bb18-8ab64fd0b181/ARS30X-Datasheet-EN.pdf.pdf)
  - [Delphi radar](https://autonomoustuff.com/product/delphi-esr-2-5-24v/)

Why ROS? A convenient messaging middleware that enables different blocks of software to intercommunicate (e.g. via shared memory or IPC).

Some acronyms:
- Advanced Driving Assistance System (ADAS)
- Parking Assistance System (PAS)
- Laser Measurement System (LMS)
- Inter-Process Communication (IPC)
  - e.g. [*Lightweight Communications and Marshalling* (LCM)](https://lcm-proj.github.io/)

---

---
## General Notes


---