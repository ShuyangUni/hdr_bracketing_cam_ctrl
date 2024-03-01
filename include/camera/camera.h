/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-01 22:45:20
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-28 23:21:24
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <Spinnaker.h>
#include <ros/ros.h>

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "include/common.h"
#include "include/spinnaker_encap.h"
#include "include/event_handler/event_handler.h"

namespace hdr_attr_ctrl {
class Camera {
 public:
  virtual void Run() = 0;
  void Release() { ReleaseCamera(cam_); }

 protected:
  ros::NodeHandle nh_;
  Spinnaker::CameraPtr cam_;
  std::shared_ptr<EventHandler> controller_;
};
}  // namespace hdr_attr_ctrl
