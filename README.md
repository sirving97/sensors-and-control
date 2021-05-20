# sensors-and-control code description

Attached is all of the code used for our project. Short descriptions of each part of the code are included. 



# Group Member Contributions:

Sean Irving: 75%

Richard Schadeberg: 20%

Yicong Zhao: 5%

Plus plenty of help from friends and other group members.



# ar_track_usb_cam.launch

This file is the modified launch file for the ar track toolbox. This file is called when the AR track toolbox ros node is started. As the intel camera is pre-calibrated, the modifications that needed to be made to this launch file included, 

- entering the correct ar tag size so that the camera recorded the poses in real units (meters in this case)
- changing the image topic to the correct topic that was published by the camera 
- changing the frame topic to be the camera frame



# DobotControl.m

This class has been created as an addition to the dobot magician class to make the control functions easier to understand and more intuitive to access. This class was created with the help of my robotics group including Conner Eyles and Josh Poot for our assignment in 41013 Robotics this semester. 



# DobotSimulation.m

This class is used for simulating the dobot robot and practicing the robot's movements in MATLAB. It includes custom fkine and ikine functions that better match the joint restraints of the dobot. This class was also created for my robotics assignment this semester with the help of Josh Poot and Conner Eyles. 



# SensorsAndControlScript.m

This is the main script for the assignment. It includes performing the hand eye calibration for the dobot based on the pose of an ar tag that is visible to the camera. It also includes two methods of control that were used to perform the robot movement, one method uses transforms and matrix mulitplication and the other just uses the difference in the x, y and z directions. Both of the control methods move the dobot by sending cartesian coordinates in the robot base frame. 


# ar_convert package
This package is used to take the ar tag poses that are published to ros and publish them to another topic that can be subscribed to in matlab. This is needed because we had problems subscribing to the original pose topic that was supplied by the ar track toolbox. 
NOTE: WE DID NOT WRITE THIS. THIS PACKAGE WAS WRITTEN BY HIMAVAAN CHANDRA AND WE HAVE SIMPLY REUSED IT. PLEASE DO NOT MARK US ON THIS CODE AS WE DO NOT OWN IT.


# Steps to run this code

You will need the dobot magician driver for linux, the realsense2 ros driver, the ar_tag_toolbox ros package and the ar_convert package.

1 Connect the dobot and the camera
2 Run the realsense camera using 

	roslaunch realsense2_camera rs_camera.launch

3 Run the ar tag toolbox with the modified launch file in the launch folder using 

	roslaunch ar_tag_toolbox ar_track_usb_cam.launch

4 Next run the ar_convert package by calling

	rosrun ar_convert main

5 you should see a display of all of the visible tag numbers in the terminal

6 Run the dobot driver using 

	roslaunch dobot_magician_driver dobot_magician.launch

7 In the sensors and control main script I have commented in each individual section what sections need to be run and in which order. 

If you need any help running the code or would like any further explanation on the operation of the code. Don't hesitate to contact me. 

--Sean
