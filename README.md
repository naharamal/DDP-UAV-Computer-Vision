# DDP-UAV-Computer-Vision

## Changelog

25/6/21 Added Camera logging and keyboard control
2/7/21 Added Object tracking by bounding box drawing

7/8/21 Added Object circling, video object tracking

The primary aim of the project is to design and test out various vision based control tasks for a quadcopter. For this purpose Microsoft [AirSim](https://microsoft.github.io/AirSim/) was chose as the simulation platform.

## AirSim

AirSim is a simulator for drones, cars and more, built on Unreal Engine . It is open-source, cross platform, and supports software-in-the-loop simulation with popular flight controllers such as PX4 & ArduPilot and hardware-in-loop with PX4 for physically and visually realistic simulations.

Python API is available for controlling the drone in an Unreal environment using Airsim. We used this functionality to design and test vision based algorithms.

## Progress

### Keyboard Controlling

A python script for keyboard control of AirSim quadcopter was written. 

### Object Tracking

A python script was used to acquire real-time images from the drone cameras. Using OpenCV object tracking functionalities a method has been implemented to select an object in the frame and track the box using a bounding box.

We used the medianflow tracker in opencv.

### Object Centering routine

After detecting the object, with the tracking functionality we can center the object in the frame by moving the drone. For this A PD controller was designed. We considered three types of errors and three modes of control inputs. 

![DDP-UAV-Computer-Vision%20cfb8438721d2449fb7873ae4055a8fbb/WhatsApp_Image_2021-08-07_at_4.25.09_PM.jpeg](DDP-UAV-Computer-Vision%20cfb8438721d2449fb7873ae4055a8fbb/WhatsApp_Image_2021-08-07_at_4.25.09_PM.jpeg)

                                                             $y_e = y_0 - y_1$

                                                                          $x_e = x_0 - x_1$

                                                $a_e = area_{frame} - area_{bb}$ 

These errors are defined and it was driven to zero using a PD control. The error in y direction is controlled by controlling the velocity in z direction of UAV. Likewise x error is controlled using yaw rate. Forward velocity is commanded to control the area error.

[DDP-UAV-Computer-Vision%20cfb8438721d2449fb7873ae4055a8fbb/obj_center.mp4](DDP-UAV-Computer-Vision%20cfb8438721d2449fb7873ae4055a8fbb/obj_center.mp4)

## Object Following

After object centering a rudimentary object pursuit algorithm was tried. If the object starts moving quadcopter will try to follow the object. this was made possible by using the centering algorithm itself.

## Object Localization

After object centering we can command a constant side velocity to the drone so that drone will circle around the object. We used tld tracker for this task as medianflow failed to detect object as we go around the object. Using the Yaw rate and the constant side velocity we could write the radius of the circle as

$Velocity/yaw = radius$  

From this radius measurement we could write estimate the position of the object in the world frame.

Same radius can be estimated geometrically from the trajectory of the quadcopter.

Below are the trajectory of the UAV on XY plane for two different simulations.

![DDP-UAV-Computer-Vision%20cfb8438721d2449fb7873ae4055a8fbb/circle_sim.png](DDP-UAV-Computer-Vision%20cfb8438721d2449fb7873ae4055a8fbb/circle_sim.png)

![DDP-UAV-Computer-Vision%20cfb8438721d2449fb7873ae4055a8fbb/circle_sim1.png](DDP-UAV-Computer-Vision%20cfb8438721d2449fb7873ae4055a8fbb/circle_sim1.png)