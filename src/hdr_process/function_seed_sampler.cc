/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-25 14:49:54
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-25 05:11:17
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/hdr_process/function_seed_sampler.h"

#include <chrono>  // NOLINT

namespace hdr_attr_ctrl {

FunctionSeedSampler::FunctionSeedSampler(
    const CameraResponseFunction &crf, const int &n_seed,
    const double &range_ratio, const double &sigma_ratio,
    const double &gap_expo_val_min, const double &gap_expo_val_max,
    const int &resize_h, const int &resize_w) {
  crf_ = crf;
  n_seed_ = n_seed;
  range_ratio_ = range_ratio;
  sigma_ratio_ = sigma_ratio;
  gap_expo_val_min_ = gap_expo_val_min;
  gap_expo_val_max_ = gap_expo_val_max;
  resize_h_ = resize_h;
  resize_w_ = resize_w;
}

void FunctionSeedSampler::Process(const cv::Mat &img, const Exposure &expo,
                                  FuncSamples *seeds) {
  // preprocess input
  seeds->clear();
  Exposure expo_aft_offset = expo;
  expo_aft_offset.expo_t;
  double expo_val = Exposure2ExposureVal(expo_aft_offset);

  double rl = 0.1875;
  double rh = 1.0 - rl;
  cv::Rect roi(static_cast<int>(img.cols * rl),
               static_cast<int>(img.rows * rl),
               static_cast<int>(img.cols * (rh - rl)),
               static_cast<int>(img.rows * (rh - rl)));
  cv::Mat img_trim = img(roi);
  ROS_INFO("%d, %d, %d, %d", roi.x, roi.y, roi.width, roi.height);
  cv::Mat img_resize;
  ImagePreprocess(img_trim, &img_resize);
  // cv::Mat img_resize;
  // ImagePreprocess(img, &img_resize);

  // add first (raw) image
  FuncSample seed_cur;
  seed_cur.expo_val = expo_val;
  seed_cur.score = CalcImageScore(img_resize);
  seed_cur.sigma = 0.0;
  seeds->push_back(seed_cur);

  // generate seeds
  FuncSamples seeds_synthetic;
  GenerateSeeds(expo_val, gap_expo_val_min_, gap_expo_val_max_,
                &seeds_synthetic);

  // calculate synthetic image metrics
  Eigen::MatrixXf irr_eigen;
  crf_.Image2Irradiance(img_trim, expo_val, &irr_eigen);
  // crf_.Image2Irradiance(img, expo_val, &irr_eigen);
  // std::cout << irr_eigen << std::endl;
  cv::Mat img_synthetic;
  for (size_t i = 0; i < seeds_synthetic.size(); ++i) {
    crf_.Irradiance2Image(irr_eigen, seeds_synthetic.at(i).expo_val,
                          &img_synthetic);
    ImagePreprocess(img_synthetic, &img_resize);
    seeds_synthetic.at(i).score = CalcImageScore(img_resize);
    seeds->push_back(seeds_synthetic.at(i));
  }
}

void FunctionSeedSampler::ImagePreprocess(const cv::Mat &img_in,
                                          cv::Mat *img_out) {
  cv::resize(img_in, *img_out, cv::Size(resize_w_, resize_h_));
}

void FunctionSeedSampler::GenerateSyntheticImage(const cv::Mat &img_in,
                                                 const double &expo_val_in,
                                                 const double &expo_val_out,
                                                 cv::Mat *img_out) {
  double ln_expo_val_in = log(expo_val_in);
  double ln_expo_val_out = log(expo_val_out);
}

double FunctionSeedSampler::CalcImageScore(const cv::Mat &img) {
  cv::Mat gm = CalcImageGradientMagnitude(img);
  return cv::mean(gm)[0];
}

void FunctionSeedSampler::ShowSampleElements(const FuncSamples &elements) {
  // sorted by x range
  FuncSamples samples;
  samples.assign(elements.begin(), elements.end());
  sort(samples.begin(), samples.end(),
       [](FuncSample x, FuncSample y) { return x.expo_val < y.expo_val; });

  for (size_t i = 0; i < samples.size(); ++i) {
    std::cout << "id " << i << " - expo_val: " << samples.at(i).expo_val
              << ", score: " << samples.at(i).score
              << ", sigma: " << samples.at(i).sigma << std::endl;
  }
}

void FunctionSeedSampler::GenerateSeeds(const double &expo_val,
                                        const double &gap_expo_val_min,
                                        const double &gap_expo_val_max,
                                        FuncSamples *seeds) {
  seeds->clear();
  int n_seed = n_seed_;
  double expo_s = expo_val;
  double expo_e = expo_val * range_ratio_;
  double sample_range = expo_e - expo_s;
  if (sample_range > n_seed * gap_expo_val_max) {
    sample_range = n_seed * gap_expo_val_max;
  }
  if (sample_range < n_seed * gap_expo_val_min) {
    n_seed = ceil(sample_range / gap_expo_val_min);
    sample_range = n_seed * gap_expo_val_min;
  }
  double sample_gap = sample_range / n_seed;
  for (size_t i = 1; i <= n_seed; ++i) {
    FuncSample seed;
    seed.expo_val = i * sample_gap + expo_s;
    seed.sigma =
        (seed.expo_val - expo_val) / (expo_e - expo_val) * sigma_ratio_;
    seeds->push_back(seed);
  }
}

}  // namespace hdr_attr_ctrl
