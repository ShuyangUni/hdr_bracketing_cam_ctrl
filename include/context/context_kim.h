/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-28 14:41:40
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-09 15:48:53
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once
#include <string>

#include "include/common.h"
#include "include/context/context.h"

namespace hdr_attr_ctrl {

struct ContextKim : public Context {
  // ctrl attributes
  double kim_seed_expo_t;
  double kim_seed_gain;
  double kim_gp_hyper_c;
  double kim_gp_hyper_l;
  double kim_gp_hyper_sn;
  double kim_gp_interval_expo_t;
  double kim_gp_interval_gain;
  std::string kim_g_func_filepath;
  int kim_bo_max_iter;
  double kim_bo_alpha;
  int kim_img_resize_w;
  int kim_img_resize_h;

  void Init(const YAML::detail::iterator_value &cam) {
    InitGeneral(cam);
    kim_seed_expo_t = cam["kim_seed_expo_t"].as<double>();
    kim_seed_gain = cam["kim_seed_gain"].as<double>();
    kim_gp_interval_expo_t = cam["kim_gp_interval_expo_t"].as<double>();
    kim_gp_interval_gain = cam["kim_gp_interval_gain"].as<double>();
    kim_gp_hyper_c = cam["kim_gp_hyper_c"].as<double>();
    kim_gp_hyper_l = cam["kim_gp_hyper_l"].as<double>();
    kim_gp_hyper_sn = cam["kim_gp_hyper_sn"].as<double>();
    kim_g_func_filepath = cam["kim_g_func_filepath"].as<std::string>();
    kim_bo_max_iter = cam["kim_bo_max_iter"].as<int>();
    kim_bo_alpha = cam["kim_bo_alpha"].as<double>();
    kim_img_resize_w = cam["kim_img_resize_w"].as<int>();
    kim_img_resize_h = cam["kim_img_resize_h"].as<int>();
  }

  void Show() {
    ROS_INFO("--------------------CAMERA CONTEXT--------------------");
    ROS_INFO("Context Type: ContextHDR");
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
    ROS_INFO("KIM Seed Image Exposure Time: %.1f", kim_seed_expo_t);
    ROS_INFO("KIM Seed Image Gain: %.1f", kim_seed_gain);
    ROS_INFO("KIM Gaussian Process Exposure Time Interval: %.1f",
             kim_gp_interval_expo_t);
    ROS_INFO("KIM Gaussian Process Gain Interval: %.1f", kim_gp_interval_gain);
    ROS_INFO("KIM Gaussian Process Hyper C: %.1f", kim_gp_hyper_c);
    ROS_INFO("KIM Gaussian Process Hyper L: %.1f", kim_gp_hyper_l);
    ROS_INFO("KIM Gaussian Process Hyper Seed Noise: %.1f", kim_gp_hyper_sn);
    ROS_INFO("KIM Camera Response Function Filepath: %s",
             kim_g_func_filepath.c_str());
    ROS_INFO("KIM Bayesian Optimization Maximum Iteration: %d",
             kim_bo_max_iter);
    ROS_INFO("KIM Bayesian Optimization Alpha: %.1f", kim_bo_alpha);
    ROS_INFO("KIM Image Resize Width: %d", kim_img_resize_w);
    ROS_INFO("KIM Image Resize Height: %d", kim_img_resize_h);
    ROS_INFO("------------------------------------------------------");
  }
};
}  // namespace hdr_attr_ctrl
