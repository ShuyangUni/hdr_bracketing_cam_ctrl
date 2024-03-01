/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-24 15:35:34
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-08 18:53:40
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
#include <vector>

#include <opencv2/opencv.hpp>

#include "include/common.h"
#include "include/context/contexts.h"
#include "include/event_handler/event_handler.h"
#include "include/polynomial.h"
#include "include/spinnaker_encap.h"

namespace hdr_attr_ctrl {
class EventHandlerShim : public EventHandler {
 public:
  EventHandlerShim(const ros::NodeHandle &nh, const Spinnaker::CameraPtr &cam,
                   const std::shared_ptr<ContextShim> &context);

  void OnImageEvent(Spinnaker::ImagePtr image);

 private:
  std::shared_ptr<ContextShim> context_;
  cv::Mat img_cv_;
  Exposure expo_cur_;

  std::vector<double> gamma_seeds_;
  std::vector<double> gamma_samples_;
  std::vector<cv::Mat> lut_list_;

  void GenerateGammaCorrectionLUT(cv::Mat *lut, const double &gamma);
  double ExposureUpdate(const double &expo_val_cur, const double &gamma);
};
}  // namespace hdr_attr_ctrl
