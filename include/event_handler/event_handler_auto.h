/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-19 15:36:07
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-09 17:08:36
 * @Description:
 *
 * Copyright (c) 2022 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"
#include "include/common.h"
#include "include/context/contexts.h"
#include "include/event_handler/event_handler.h"

namespace hdr_attr_ctrl {
class EventHandlerAuto : public EventHandler {
 public:
  EventHandlerAuto(const ros::NodeHandle &nh, const Spinnaker::CameraPtr &cam,
                   const std::shared_ptr<ContextAuto> &context);

  void OnImageEvent(Spinnaker::ImagePtr image);

 private:
  std::shared_ptr<ContextAuto> context_;
  Exposure expo_;
};

}  // namespace hdr_attr_ctrl
