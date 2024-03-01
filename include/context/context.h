/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-28 14:29:01
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-09 15:48:19
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <ros/ros.h>

#include <string>

#include "3rdparty/yaml-cpp/include/yaml-cpp/yaml.h"
#include "include/common.h"

namespace hdr_attr_ctrl {

struct Context {
  // basic information
  std::string cam_ns;
  int cam_serial;
  CaptureMode cam_cap_mode;
  std::string cam_alias;
  std::string cam_sub_ns;
  // attributes range
  Range range_expo_t_major;
  Range range_gain_major;
  // exposure value offset
  double expo_val_offset;
  // rostopic
  std::string ros_topic_name_image_raw;
  bool publish_all_images;

  virtual void Init(const YAML::detail::iterator_value &cam) = 0;
  virtual void Show() = 0;

  void InitGeneral(const YAML::detail::iterator_value &cam) {
    // strings
    cam_alias = cam["alias"].as<std::string>();
    cam_serial = cam["serial"].as<int>();
    cam_ns = cam["ns"].as<std::string>();
    cam_sub_ns = cam["sub_ns"].as<std::string>();
    cam_cap_mode = String2CaptureMode(cam["mode"].as<std::string>());
    // ranges
    range_expo_t_major.first = cam["expo_t_major_lower"].as<double>();
    range_expo_t_major.second = cam["expo_t_major_upper"].as<double>();
    range_gain_major.first = cam["gain_major_lower"].as<double>();
    range_gain_major.second = cam["gain_major_upper"].as<double>();
    // exposure value offset
    expo_val_offset = cam["expo_val_offset"].as<double>();
    publish_all_images = cam["publish_all_images"].as<bool>();
    // rostopic name
    ros_topic_name_image_raw = "/" + cam_ns + "/" + cam_sub_ns + "/image_raw";
  }
};

}  // namespace hdr_attr_ctrl
