/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-25 16:18:54
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-29 01:03:34
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/hdr_process/camera_response_function.h"

namespace hdr_attr_ctrl {

CameraResponseFunction::CameraResponseFunction() {}

CameraResponseFunction::CameraResponseFunction(
    const std::string &str_func_coeff) {
  LoadFunctionCoeff(str_func_coeff);
  InitFunctions();
  g_func_.Inference(0.0, &irr_min_);
  g_func_.Inference(255.0, &irr_max_);
}

void CameraResponseFunction::Image2Irradiance(const cv::Mat &img,
                                              const double &expo_val,
                                              Eigen::MatrixXf *irr_eigen) {
  float ln_expo_val = static_cast<float>(log(expo_val));
  Eigen::MatrixXf img_eigen;
  cv::cv2eigen(img, img_eigen);
  g_func_.Inference(img_eigen, irr_eigen);
  irr_eigen->array() -= ln_expo_val;
}

void CameraResponseFunction::Irradiance2Image(const Eigen::MatrixXf &irr_eigen,
                                              const double &expo_val,
                                              cv::Mat *img_out) {
  float ln_expo_val = static_cast<float>(log(expo_val));
  int rows = irr_eigen.rows();
  int cols = irr_eigen.cols();
  Eigen::MatrixXf irr_eigen_t = irr_eigen.array() + ln_expo_val;
  irr_eigen_t = (irr_eigen_t.array() > irr_max_).select(irr_max_, irr_eigen_t);
  irr_eigen_t = (irr_eigen_t.array() < irr_min_).select(irr_min_, irr_eigen_t);
  Eigen::MatrixXf img_eigen;
  g_inv_func_.Inference(irr_eigen_t, &img_eigen);
  cv::eigen2cv(img_eigen, *img_out);
  img_out->convertTo(*img_out, CV_8U);
}

void CameraResponseFunction::GenerateSyntheticImage(const cv::Mat &img_in,
                                                    const double &expo_val_in,
                                                    const double &expo_val_out,
                                                    cv::Mat *img_out) {
  Eigen::MatrixXf irr_eigen;
  Image2Irradiance(img_in, expo_val_in, &irr_eigen);
  Irradiance2Image(irr_eigen, expo_val_out, img_out);
}

void CameraResponseFunction::InitFunctions() {
  std::vector<double> g_func_x;
  for (size_t i = 0; i < 256; ++i) g_func_x.push_back(i);
  g_func_.Fitting(g_func_x, g_func_y_);
  g_inv_func_.Fitting(g_func_y_, g_func_x);
}

void CameraResponseFunction::LoadFunctionCoeff(
    const std::string &str_func_coeff) {
  std::ifstream ifs(str_func_coeff.c_str());
  if (!ifs.is_open()) {
    ROS_ERROR("cannot open file %s", str_func_coeff.c_str());
    ros::shutdown();
  }
  std::string line;
  g_func_y_.clear();
  while (getline(ifs, line)) {
    std::stringstream ss(line);
    double val;
    ss >> val;
    g_func_y_.push_back(val);
  }
  ifs.close();
}

}  // namespace hdr_attr_ctrl
