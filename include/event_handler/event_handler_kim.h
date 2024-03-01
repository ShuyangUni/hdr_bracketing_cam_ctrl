/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-24 15:35:34
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-25 17:02:21
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
#include "include/hdr_process/camera_response_function.h"
#include "include/hdr_process/gp_regressor.h"
#include "include/polynomial.h"
#include "include/spinnaker_encap.h"

namespace hdr_attr_ctrl {
class EventHandlerKim : public EventHandler {
 public:
  EventHandlerKim(const ros::NodeHandle &nh, const Spinnaker::CameraPtr &cam,
                   const std::shared_ptr<ContextKim> &context);

  void OnImageEvent(Spinnaker::ImagePtr image);

 private:
  std::shared_ptr<ContextKim> context_;
  cv::Mat img_cv_;
  Exposure expo_cur_;
  Exposure expo_tar_;

  std::shared_ptr<CameraResponseFunction> crf_;
  std::shared_ptr<GPRegressor> gpr_;

  bool b_wait_seed_;
  bool b_first_after_seed_;

  double psi_;
  Eigen::VectorXd px_;

  double metric_opt_;
  double metric_cur_;

  double ProcessSeed(const cv::Mat &img_seed, const double &expo_val_seed);
  bool NeedChangeExposure();

  void GPPredict(const Eigen::VectorXd &tx, const Eigen::VectorXd &ty,
                 const Eigen::VectorXd &px, Eigen::VectorXd *py,
                 Eigen::VectorXd *pvar);
  double FindQueryPoint(const Eigen::VectorXd &px, const Eigen::VectorXd &py,
                        const Eigen::VectorXd &pvar);
  void GeneratePx();

  double CalcMetricKim(const cv::Mat &img);
  void CalcWeightMask(const cv::Mat &entropy, cv::Mat *wmask);
  void CalcGrad(const cv::Mat &img, cv::Mat *grad);
  double CalcImgGours(const cv::Mat &gour);
};
}  // namespace hdr_attr_ctrl
