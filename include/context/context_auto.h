/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-28 14:29:01
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-09 15:48:41
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <string>

#include "include/common.h"
#include "include/context/context.h"

namespace hdr_attr_ctrl {

struct ContextAuto : public Context {
  void Init(const YAML::detail::iterator_value &cam) { InitGeneral(cam); }

  void Show() {
    ROS_INFO("--------------------CAMERA CONTEXT--------------------");
    ROS_INFO("Context Type: ContextAuto");
    ROS_INFO("Camera Serial: %d", cam_serial);
    ROS_INFO("Camera Alias: %s", cam_alias.c_str());
    ROS_INFO("Camera Namespace: %s", cam_ns.c_str());
    ROS_INFO("Camera Subnamespace: %s", cam_sub_ns.c_str());
    ROS_INFO("Camera Capture Mode: %s",
             CaptureMode2String(cam_cap_mode).c_str());
    ROS_INFO("ExposureTime Range: [%.1f, %.1f]", range_expo_t_major.first,
             range_expo_t_major.second);
    ROS_INFO("Gain Range: [%.1f, %.1f]", range_gain_major.first,
             range_gain_major.second);
    ROS_INFO("Image ROS Topic Name: %s", ros_topic_name_image_raw.c_str());
    ROS_INFO("Whether Publish all Images: %d", publish_all_images);
    ROS_INFO("------------------------------------------------------");
  }
};

}  // namespace hdr_attr_ctrl
