# indoor_navigation_system

In this pproject i have built the any system using camera that allows our system to know the excet location of our robot in a define arena. Arena is a predefine area whos corners are marked using Aruco marker. We have same mark on the robot too, the continueous processing on the feed of the camera is done to know the location of the robot. System is built in the ROS2 enviornment and the robot contain ESP32 as the microcontroller board, so we sended orentation and linear movment coordinates to the robt over wifi. All the computation is done on the laptop as we have created a local network over wifi for the data transfer.

This project implements an indoor autonomous navigation system using ROS2. 
All processing is done on a laptop, and movement commands are sent to the robot.
