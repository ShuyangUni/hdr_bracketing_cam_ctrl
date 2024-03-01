/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-18 14:23:39
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-08 20:17:23
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
#include "include/context/contexts.h"
#include "include/event_handler/event_handlers.h"
#include "include/spinnaker_encap.h"

extern std::vector<std::shared_ptr<hdr_attr_ctrl::Context>> g_context_list;

namespace hdr_attr_ctrl {

class CameraKim : public Camera {
 public:
  CameraKim(const Spinnaker::CameraPtr &cam_ptr, const ros::NodeHandle &nh,
               const size_t &global_idx);
  void Run();

 private:
  std::shared_ptr<ContextKim> context_;

  void InitCameraForManualExposure();
  void RegisterController();
  void InitCameraAttribute();
};

}  // namespace hdr_attr_ctrl
