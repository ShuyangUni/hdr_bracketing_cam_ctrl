/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-25 14:49:54
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-29 01:36:21
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>

#include "include/common.h"
#include "include/hdr_process/camera_response_function.h"

namespace hdr_attr_ctrl {
class FunctionSeedSampler {
 public:
  FunctionSeedSampler(const CameraResponseFunction &crf, const int &n_seed,
                      const double &range_ratio, const double &sigma_ratio,
                      const double &gap_expo_val_min,
                      const double &gap_expo_val_max, const int &resize_h,
                      const int &resize_w);
  void Process(const cv::Mat &img, const Exposure &expo, FuncSamples *seeds);

  double CalcImageScore(const cv::Mat &img);
  void ShowSampleElements(const FuncSamples &elements);

 private:
  double expo_offset_;
  CameraResponseFunction crf_;
  int n_seed_;
  double range_ratio_;
  double sigma_ratio_;
  double gap_expo_val_min_;
  double gap_expo_val_max_;
  int resize_h_;
  int resize_w_;

  void ImagePreprocess(const cv::Mat &img_in, cv::Mat *img_out);
  void GenerateSyntheticImage(const cv::Mat &img_in, const double &expo_val_in,
                              const double &expo_val_out, cv::Mat *img_out);
  void GenerateSeeds(const double &expo_val, const double &gap_expo_val_min,
                     const double &gap_expo_val_max, FuncSamples *seeds);
};
}  // namespace hdr_attr_ctrl
