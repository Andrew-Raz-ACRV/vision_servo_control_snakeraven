# SnakeRaven Visual Servoing
SnakeRaven with endoscopic camera ROS communication and kinematics code for the RAVEN II robotic platform

SnakeRaven is an instrument that can be attached to the Raven II and controlled via keyboard teleoperation.
This version implements an endoscopic camera and allows it to control the orientation of the robot.

This software primarily contains:
- ROS nodes for controlling SnakeRaven when attached to the Raven II
- ROS nodes for a vision system that would take raw images and compute the control action
- A class with functions that solves the forward and inverse kinematics of SnakeRaven in C++

![alt text](https://github.com/Andrew-Raz-ACRV/vision_servo_control_snakeraven/blob/main/rqt_graph_diagram.png)

## Prerequisite installation :
The vision_servo_control_snakeraven uses Eigen to compute the kinematics and control algorithms. To install this do:
1. Go to http://eigen.tuxfamily.org/index.php?title=Main_Page#Download to get the most recent Eigen. 
2. Download the zip, extract and find the subfolder "Eigen" 
3. In snake_raven_controller/ and vision_system_snakeraven create the folder 'include' and paste 'Eigen' into the include folder.

## Contact

This code is written by Andrew Razjigaev. If there are queries you can contact him via email: andrew_razjigaev@outlook.com

## Relavent links:
1. **snake_raven_controller** : This is an expansion on from the original [controller](https://github.com/Andrew-Raz-ACRV/snake_raven_controller)
