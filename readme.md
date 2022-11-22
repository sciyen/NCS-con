# NCS-CON: Multi-view Multi-markers State Estimation System

## Introduction
The core pose estimation algorithm is an migration from [HSarham/automatic-ar](https://github.com/HSarham/automatic-ar). 
We provide a ROS interface to receive marker detections from multiple camera devices, and one can test different pose estimation algorithms by taking the advantages of ROS.

## System architecture
![](https://i.imgur.com/2bsuxUg.png)

![](https://i.imgur.com/TIBDqDe.png)

## Usage
1. For the calibration of pi-cameras, please see [Calibration](./src/calibration/readme.md).
2. To launch the pose_estimation system, please see [Pose Estimation](./ros_ws/readme.md)

## References

1. [H. Sarmadi, R. Muñoz-Salinas, M. A. Berbís and R. Medina-Carnicer, "Simultaneous Multi-View Camera Pose Estimation and Object Tracking With Squared Planar Markers," in IEEE Access, vol. 7, pp. 22927-22940, 2019.](https://doi.org/10.1109/ACCESS.2019.2896648)