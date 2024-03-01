/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-18 14:23:39
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-31 15:04:03
 * @Description:
 *
 * Copyright (c) 2022 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <Spinnaker.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "include/camera/camera.h"
#include "include/common.h"
#include "include/context/contexts.h"
#include "include/event_handler/event_handlers.h"
#include "include/spinnaker_encap.h"

extern std::vector<std::shared_ptr<hdr_attr_ctrl::Context>> g_context_list;

namespace hdr_attr_ctrl {

class CameraAuto : public Camera {
 public:
  CameraAuto(const Spinnaker::CameraPtr &cam_ptr, const ros::NodeHandle &nh,
             const size_t &global_idx);
  void Run();

 private:
  std::shared_ptr<ContextAuto> context_;

  void InitCameraForAutoExposure();
  void RegisterController();
};

}  // namespace hdr_attr_ctrl
