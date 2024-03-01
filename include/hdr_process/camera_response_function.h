/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-25 16:18:54
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-29 01:02:14
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "include/polynomial.h"

namespace hdr_attr_ctrl {
class CameraResponseFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraResponseFunction();
  explicit CameraResponseFunction(const std::string &str_func_coeff);

  void GenerateSyntheticImage(const cv::Mat &img_in, const double &expo_val_in,
                              const double &expo_val_out, cv::Mat *img_out);
  void Image2Irradiance(const cv::Mat &img, const double &expo_val,
                        Eigen::MatrixXf *irr_eigen);
  void Irradiance2Image(const Eigen::MatrixXf &irr_eigen,
                        const double &expo_val, cv::Mat *img_out);

 private:
  Polynomial<10> g_func_;
  Polynomial<10> g_inv_func_;
  std::vector<double> g_func_y_;

  double irr_min_, irr_max_;

  void LoadFunctionCoeff(const std::string &str_func_coeff);
  void InitFunctions();
};
}  // namespace hdr_attr_ctrl
