# Optical_Flow_Sensor
Repo For Optical Flow Sensor

![Optical Flow](Optical_flow.jpg)

The ROS 2 Optical Flow Sensor package provides **real-time motion estimation** by analyzing the **displacement of visual features between consecutive camera frames**. Using computer vision techniques implemented with OpenCV, the package calculates optical flow vectors and publishes the resulting motion estimates as ROS 2 topics.

Optical flow is widely used in robotics to estimate the relative movement of a robot or camera without relying solely on GPS or wheel encoders. This package can be integrated into mobile robots, drones, autonomous vehicles, and research platforms requiring visual motion estimation.

***How to use it:*** 

The functionality of an optical flow sensor of course depends on being able to **find features to track**, a surface that is very uniform will be hard to track since all the frames will look the same. If you’ve ever tried using a mouse on a glass table or reflective surface you’ve probably seen that it doesn’t work.

## How it works : 



