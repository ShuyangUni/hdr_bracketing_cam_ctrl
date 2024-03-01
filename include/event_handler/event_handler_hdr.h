/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-24 15:35:34
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-09 17:07:44
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "include/common.h"
#include "include/context/contexts.h"
#include "include/event_handler/event_handler.h"
#include "include/hdr_process/camera_response_function.h"
#include "include/hdr_process/function_seed_sampler.h"
#include "include/hdr_process/gp_regressor.h"
#include "include/spinnaker_encap.h"

namespace hdr_attr_ctrl {

class EventHandlerHDR : public EventHandler {
  static constexpr int k_expo_idle_ = 0;
  static constexpr int k_expo_major_ = 1;
  static constexpr int k_expo_low_ = 2;
  static constexpr int k_expo_high_ = 4;

 public:
  EventHandlerHDR(const ros::NodeHandle &nh, const Spinnaker::CameraPtr &cam,
                  const std::shared_ptr<ContextHDR> &context);

  void OnImageEvent(Spinnaker::ImagePtr image);

 private:
  std::shared_ptr<ContextHDR> context_;
  std::shared_ptr<FunctionSeedSampler> seed_sampler_;
  std::shared_ptr<GPRegressor> gpr_;

  cv::Mat img_cur_;
  Exposure expo_cur_;
  FuncSamples seeds_cur_;

  bool valid_major_;
  bool valid_low_;
  bool valid_high_;
  Exposure expo_major_;
  Exposure expo_low_;
  Exposure expo_high_;
  FuncSamples seeds_major_;
  FuncSamples seeds_low_;
  FuncSamples seeds_high_;
  double expo_val_tar_major_;
  double score_tar_major_;

  Range range_expo_t_major_;
  Range range_gain_major_;
  Range range_expo_t_low_;
  Range range_gain_low_;
  Range range_expo_t_high_;
  Range range_gain_high_;

  ros::Publisher pub_image_full_;

  void InitCtrl();

  ExpoState FindExpoState(const Exposure &expo);
  bool UpdateSeedsByExpoState(const ExpoState &expo_state,
                              const Exposure &expo_cur,
                              const FuncSamples &seeds_cur);
  bool AssembleSeeds(const ExpoState &expo_state, FuncSamples *seeds_train);

  void FindBestExposure(const FuncSamples &seeds_predict, double *expo_val_opt,
                        double *score_opt);
  bool JudgeWhetherUpdate(const double &expo_val_cur, const double &score_cur);
  void UpdateSequenceStates(SequenceStates *states);

  void HandleAmbiguity(Exposure *expo, const Range &range_expo_t);
};
}  // namespace hdr_attr_ctrl
