<!--
 * @Author: Shuyang Zhang
 * @Date: 2024-03-01 16:46:19
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2024-03-01 19:16:01
 * @Description: 
 * 
 * Copyright (c) 2024 by Shuyang Zhang, All Rights Reserved. 
-->
# Parameter Configurations
* We need to prepare two files to run this driver.
  * a YAML file to manage our parameters. The YAML files are in the [param](../param/) folder.
  * a ROS launch file to configure ROS node and pass the YAML file with its argument. The launch files are in the [launch](../launch/) folder.
* The YAML files are used by ([YAML-CPP](https://github.com/jbeder/yaml-cpp)). The structure of the YAML file is as follows:
```
├── Camera
    ├── ns
    ├── serial 
    ├── mode
    ├── alias
    ├── sub_ns
    ├── expo_t_major_lower
    ├── expo_t_major_upper
    ├── gain_major_lower
    ├── gain_major_upper
    ├── expo_val_offset
    .
    .
```
* **Camera**: camera list, this driver can use multiple cameras at the same time to run different methods. If we have 4 cameras to run different methods, we need to generate 4 camera instance with different method setup.
* **ns**: namespace, for ROS image output topic.
* **sub_ns**: sub namespace, for ROS image output topic.
* **alias**: camera name, for ROS image output topic.
* **serial**: camera serial id, need to be obtained from camera hardware.
* **mode**: camera capture method, containing ours and 3 other baselines, including
  * **auto**: the built-in method within FLIR BFS, which sets the average intensity value as a image quality metric and uses a feedback control scheme to introduce camera attribute control.
  * **shim**: an implementation of [Shim2014IROS](https://joonyoung-cv.github.io/assets/paper/14_iros_auto_adjusting.pdf), a feedback control method with the $\gamma$-correction technique.
  * **kim**: an implementation of [Kim2020TRO](https://github.com/RPM-Robotics-Lab/cam_attr_controller). a direct method using image synthesis techiniques with Bayesian Optimiation. Since their controller is implemented via another camera platform, we re-implement their method on FLIR BFS cameras, relying on their papers and open source codes.
  * **hdr**: Our method, based on the method of Kim2020TRO, with special image bracketing capture, using image synthesis techiniques with Gaussian Process Regression.
* **expo_t_major_lower**: minimum of camera exposure time, for boundary check
* **expo_t_major_upper**: maximum of camera exposure time, for boundary check
* **gain_major_lower**: minimum of camera analog gain, for boundary check
* **gain_major_upper**: maximum of camera analog gain, for boundary check
* **expo_val_offset**: an offset of camera exposure time, there is a difference between the readout time and the actual exposure time.
* Different methods have their own extra parameters. Please follow the [test configurations](../param) for our implementations of the 4 modes.
