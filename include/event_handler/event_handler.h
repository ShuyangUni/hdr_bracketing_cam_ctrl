/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-24 15:35:34
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-09 17:08:40
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "include/common.h"

namespace hdr_attr_ctrl {
class EventHandler : public Spinnaker::ImageEventHandler {
 public:
  EventHandler(const ros::NodeHandle &nh, const Spinnaker::CameraPtr &cam) {
    nh_ = nh;
    cam_ = cam;
    seq_id_ = 0;
  }

  virtual void OnImageEvent(Spinnaker::ImagePtr image) = 0;

 protected:
  ros::NodeHandle nh_;
  Spinnaker::CameraPtr cam_;
  ros::Publisher pub_image_raw_;
  uint32_t seq_id_;
  ros::Time ts_;
};
}  // namespace hdr_attr_ctrl
