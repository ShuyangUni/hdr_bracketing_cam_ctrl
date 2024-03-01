<!--
 * @Author: Shuyang Zhang
 * @Date: 2024-02-29 17:04:28
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2024-02-29 17:41:34
 * @Description: 
 * 
 * Copyright (c) 2024 by Shuyang Zhang, All Rights Reserved. 
-->
# HDR Bracketing Camera Attribute Control

This is an official repository of

**An Image Acquisition Scheme for Visual Odometry based on Image Bracketing and Online Attribute Control**, Shuyang Zhang, Jinhao He, Bohuan Xue, Jin Wu, Pengyu Yin, Jianhao Jiao and Ming Liu.

This paper will be officially released on ICRA 2024.

![overview](https://github.com/ShuyangUni/hdr_bracketing_cam_ctrl/assets/35594134/ad15800b-cd62-40cd-8b27-a7b6e956932b)
System Overview

## Highlights
* A camera attribute control method adapted to image bracketing patterns. Images with various exposures are captured for scene exploration, and optimal exposure for the next control is globally optimized by Gaussian process regression (GPR).
* A VO-oriented image acquisition scheme that explores a wide dynamic range and provides stable image sequences in the time domain. The system leverages the exploration of the dynamic range with system output frequency according to the bracketing pattern design.

## Requirements
### Hardware
* Camera with Image Bracketing Interface. We use the [FLIR BFS cameras](https://www.flir.com/products/blackfly-s-usb3/?vertical=machine%20vision&segment=iis) (FLIR BFS-U3-32S4C) and the bracketing images are captured by the Sequencer implementation of [Spinnaker API](https://www.flir.com/products/spinnaker-sdk/?vertical=machine+vision&segment=iis).

### Software
* Ubuntu (20.04)
* [ROS (Noetic Ninjemys)](https://wiki.ros.org/noetic/Installation/Ubuntu)
* [Spinnaker (2.7.0.128)](https://www.flir.com/products/spinnaker-sdk/)
* OpenCV (installed with ROS)
* Eigen (installed with ROS)

## Getting Start

