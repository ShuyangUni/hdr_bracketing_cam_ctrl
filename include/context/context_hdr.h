/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-28 14:41:40
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-09 15:48:48
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once
#include <string>

#include "include/common.h"
#include "include/context/context.h"

namespace hdr_attr_ctrl {

struct ContextHDR : public Context {
  // ctrl attributes
  double hdr_expo_t_shift;
  Exposure hdr_expo_init_major;
  Exposure hdr_expo_init_low;
  Exposure hdr_expo_init_high;
  std::string hdr_g_func_filepath;
  int hdr_resize_w;
  int hdr_resize_h;
  double hdr_expo_val_ratio_low;
  double hdr_expo_val_ratio_high;
  double hdr_ctrl_score_diff_ratio;
  double hdr_ctrl_expo_val_diff_ratio;
  double hdr_ctrl_expo_val_diff_abs;
  // seed sampler
  int ss_n_seed;
  double ss_range_ratio;
  double ss_sigma_ratio;
  double ss_gap_expo_val_min;
  double ss_gap_expo_val_max;
  // gpr
  double gpr_hyper_c;
  double gpr_px_sample_res;
  // global control
  std::mutex hdr_mutex;
  hdr_attr_ctrl::SequenceStates hdr_sequencer_states;
  bool hdr_b_update;

  void Init(const YAML::detail::iterator_value &cam) {
    InitGeneral(cam);
    hdr_expo_t_shift = cam["hdr_expo_t_shift"].as<double>();
    hdr_expo_init_major.expo_t = cam["hdr_init_major_expo_t"].as<double>();
    hdr_expo_init_major.gain = cam["hdr_init_major_gain"].as<double>();
    hdr_expo_init_low.expo_t = cam["hdr_init_low_expo_t"].as<double>();
    hdr_expo_init_low.gain = cam["hdr_init_low_gain"].as<double>();
    hdr_expo_init_high.expo_t = cam["hdr_init_high_expo_t"].as<double>();
    hdr_expo_init_high.gain = cam["hdr_init_high_gain"].as<double>();
    hdr_g_func_filepath = cam["hdr_g_func_filepath"].as<std::string>();
    hdr_resize_w = cam["hdr_resize_w"].as<int>();
    hdr_resize_h = cam["hdr_resize_h"].as<int>();
    hdr_expo_val_ratio_low = cam["hdr_expo_val_ratio_low"].as<double>();
    hdr_expo_val_ratio_high = cam["hdr_expo_val_ratio_high"].as<double>();
    hdr_ctrl_score_diff_ratio = cam["hdr_ctrl_score_diff_ratio"].as<double>();
    hdr_ctrl_expo_val_diff_ratio =
        cam["hdr_ctrl_expo_val_diff_ratio"].as<double>();
    hdr_ctrl_expo_val_diff_abs = cam["hdr_ctrl_expo_val_diff_abs"].as<double>();
    ss_n_seed = cam["ss_n_seed"].as<int>();
    ss_range_ratio = cam["ss_range_ratio"].as<double>();
    ss_sigma_ratio = cam["ss_sigma_ratio"].as<double>();
    ss_gap_expo_val_min = cam["ss_gap_expo_val_min"].as<double>();
    ss_gap_expo_val_max = cam["ss_gap_expo_val_max"].as<double>();
    gpr_hyper_c = cam["gpr_hyper_c"].as<double>();
    gpr_px_sample_res = cam["gpr_px_sample_res"].as<double>();
    hdr_b_update = false;
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
    ROS_INFO("HDR ExposureTime Shift: %.1f", hdr_expo_t_shift);
    ROS_INFO("HDR Major Image Init Attribute: <%.1f, %.1f>",
             hdr_expo_init_major.expo_t, hdr_expo_init_major.gain);
    ROS_INFO("HDR Low Image Init Attribute: <%.1f, %.1f>",
             hdr_expo_init_low.expo_t, hdr_expo_init_low.gain);
    ROS_INFO("HDR High Image Init Attribute: <%.1f, %.1f>",
             hdr_expo_init_high.expo_t, hdr_expo_init_high.gain);
    ROS_INFO("HDR G Function Filepath: %s", hdr_g_func_filepath.c_str());
    ROS_INFO("HDR Image Resize: <%d, %d>", hdr_resize_h, hdr_resize_w);
    ROS_INFO("HDR Ratio from Major Exposure to Low Exposure: %.1f",
             hdr_expo_val_ratio_low);
    ROS_INFO("HDR Ratio from Major Exposure to High Exposure: %.1f",
             hdr_expo_val_ratio_high);
    ROS_INFO("HDR Control Score Ratio Difference: %.1f",
             hdr_ctrl_score_diff_ratio);
    ROS_INFO("HDR Control Exposure Value Ratio Difference: %.1f",
             hdr_ctrl_expo_val_diff_ratio);
    ROS_INFO("HDR Control Exposure Value Abs Difference: %.1f",
             hdr_ctrl_expo_val_diff_abs);
    ROS_INFO("SeedSampler Number of Seeds per Image: %d", ss_n_seed);
    ROS_INFO("SeedSampler Range Ratio: %.1f", ss_range_ratio);
    ROS_INFO("SeedSampler Sigma Ratio: %.1f", ss_sigma_ratio);
    ROS_INFO("SeedSampler Seed Sample Gap Range: [%.1f, %.1f]",
             ss_gap_expo_val_min, ss_gap_expo_val_max);
    ROS_INFO("GPR Hyperparameter C: %.1f", gpr_hyper_c);
    ROS_INFO("GPR Predict X Sample Resolution: %.1f", gpr_px_sample_res);
    ROS_INFO("------------------------------------------------------");
  }
};
}  // namespace hdr_attr_ctrl
