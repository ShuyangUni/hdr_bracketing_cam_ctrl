/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-18 14:23:39
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-31 15:04:08
 * @Description:
 *
 * Copyright (c) 2022 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <Spinnaker.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <memory>
#include <mutex>  // NOLINT
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

class CameraHDR : public Camera {
 public:
  CameraHDR(const Spinnaker::CameraPtr &cam_ptr, const ros::NodeHandle &nh,
            const size_t &global_idx);
  void InitMode();
  void Run();

 private:
  std::shared_ptr<ContextHDR> context_;
  void RegisterController();
  void InitSequencerStates(SequenceStates *states);
};

}  // namespace hdr_attr_ctrl
