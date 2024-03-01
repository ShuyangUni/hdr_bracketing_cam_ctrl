/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-18 14:23:39
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-28 16:20:17
 * @Description:
 *
 * Copyright (c) 2022 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <Spinnaker.h>
#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <string>
#include <thread>  // NOLINT
#include <vector>

#include "3rdparty/yaml-cpp/include/yaml-cpp/yaml.h"
#include "include/camera/cameras.h"
#include "include/common.h"
#include "include/context/contexts.h"

extern std::vector<std::shared_ptr<hdr_attr_ctrl::Context>> g_context_list;

namespace hdr_attr_ctrl {
class CaptureManager {
 public:
  CaptureManager();
  ~CaptureManager();
  void Init(const ros::NodeHandle &nh);
  void Run();

  void InitContextsFromYAML(const std::string &str_filepath_yaml);

 private:
  ros::NodeHandle nh_;
  Spinnaker::SystemPtr system_;
  Spinnaker::CameraList cam_list_;

  std::vector<std::shared_ptr<Camera>> camera_list_;
  std::vector<std::thread> camera_thread_;
  std::vector<int> serial_list_;

  void ModeInit(const size_t &global_id, const size_t &local_id);
  static void RunCameraAcquisitionThread(const std::shared_ptr<Camera> &cam);
};
}  // namespace hdr_attr_ctrl
