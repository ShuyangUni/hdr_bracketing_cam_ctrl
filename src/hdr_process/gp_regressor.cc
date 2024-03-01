/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-26 16:00:15
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-09 01:18:23
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/hdr_process/gp_regressor.h"

namespace hdr_attr_ctrl {
GPRegressor::GPRegressor(const double &hyper_c, const double &px_sample_res) {
  hyper_c2_ = pow(hyper_c, 2.0);
  px_sample_res_ = px_sample_res;
}

GPRegressor::GPRegressor(const double &hyper_c, const double &hyper_l,
                         const double &px_sample_res) {
  hyper_c2_ = pow(hyper_c, 2.0);
  hyper_l2_ = pow(hyper_l, 2.0);
  hyper_eta_ = -0.5 / hyper_l2_;
  px_sample_res_ = px_sample_res;
}

void GPRegressor::GPRInference(const FuncSamples &data_train,
                               FuncSamples *data_predict) {
  Preprocess(data_train);
  NormalizeY();
  CalcKernel(tx_, tx_, &kernel_);
  Eigen::MatrixXd diag_sigma = tsigma_.array().square().matrix().asDiagonal();
  kernel_ = kernel_ + diag_sigma;
  invkernel_ = kernel_.llt().solve(
      Eigen::MatrixXd::Identity(kernel_.rows(), kernel_.cols()));
  invkernel_ny_ = invkernel_ * ty_;
  Eigen::MatrixXd kernel_s;
  double kernel_ss;
  for (size_t i = 0; i < px_.size(); ++i) {
    CalcKernel(px_(i), tx_, &kernel_s);
    CalcKernel(px_(i), px_(i), &kernel_ss);
    py_(i) = (kernel_s * invkernel_ny_).value();
    psigma_(i) =
        kernel_ss - (kernel_s * invkernel_ * kernel_s.transpose()).value();
  }
  py_ = py_.array() * ty_std_ + ty_mean_;
  psigma_ *= ty_std_;
  Postprocess(data_predict);
}

void GPRegressor::Preprocess(const FuncSamples &data_train) {
  // sort
  FuncSamples data = data_train;
  sort(data.begin(), data.end(),
       [](FuncSample x, FuncSample y) { return x.expo_val < y.expo_val; });

  // init training data
  tx_ = Eigen::VectorXd::Zero(data_train.size());
  ty_ = Eigen::VectorXd::Zero(data_train.size());
  tsigma_ = Eigen::VectorXd::Zero(data_train.size());
  for (size_t i = 0; i < data_train.size(); ++i) {
    tx_(i) = data_train.at(i).expo_val;
    ty_(i) = data_train.at(i).score;
    tsigma_(i) = data_train.at(i).sigma;
  }
  // calc hyper_l2_ & hyper_eta_
  Eigen::VectorXd tx_diff =
      tx_.block(1, 0, tx_.size() - 1, 1) - tx_.block(0, 0, tx_.size() - 1, 1);
  hyper_l2_ = tx_diff.maxCoeff();
  hyper_l2_ = hyper_l2_ * hyper_l2_;
  hyper_eta_ = -0.5 / hyper_l2_;
  // calc px_
  double px_s = tx_(0);
  double px_e = tx_(tx_.size() - 1);
  int n_samples = static_cast<int>(ceil((px_e - px_s) / px_sample_res_));
  px_ = Eigen::VectorXd::Zero(n_samples);
  py_ = Eigen::VectorXd::Zero(n_samples);
  psigma_ = Eigen::VectorXd::Zero(n_samples);
  for (size_t i = 0; i < n_samples; ++i) {
    px_(i) = px_s + px_sample_res_ * i;
  }
}

void GPRegressor::Postprocess(FuncSamples *data_predict) {
  data_predict->clear();
  FuncSample data;
  for (size_t i = 0; i < px_.size(); ++i) {
    data.expo_val = px_(i);
    data.score = py_(i);
    data.sigma = CheckDoubleZero(psigma_(i));
    data_predict->push_back(data);
  }
}

void GPRegressor::CalcKernel(const Eigen::VectorXd &x1,
                             const Eigen::VectorXd &x2,
                             Eigen::MatrixXd *kernel) {
  *kernel =
      ((x1.replicate(1, x2.size()) - x2.replicate(1, x1.size()).transpose())
           .array()
           .square() *
       hyper_eta_)
          .exp() *
      hyper_c2_;
}

void GPRegressor::CalcKernel(const Eigen::VectorXd &x1, const double &x2,
                             Eigen::MatrixXd *kernel) {
  *kernel = ((x1.array() - x2).square() * hyper_eta_).exp() * hyper_c2_;
}

void GPRegressor::CalcKernel(const double &x1, const Eigen::VectorXd &x2,
                             Eigen::MatrixXd *kernel) {
  *kernel =
      ((x2.transpose().array() - x1).square() * hyper_eta_).exp() * hyper_c2_;
}

void GPRegressor::CalcKernel(const double &x1, const double &x2,
                             double *kernel) {
  *kernel = exp((x1 - x2) * (x1 - x2) * hyper_eta_) * hyper_c2_;
}

void GPRegressor::NormalizeY() {
  ty_mean_ = ty_.mean();
  ty_std_ = sqrt((ty_.array() - ty_mean_).square().sum() / (ty_.size() - 1));
  ty_ = (ty_.array() - ty_mean_) / ty_std_;
}

double GPRegressor::CheckDoubleZero(const double &val) {
  return fabs(val) < 1e-6 ? 0.0 : val;
}

}  // namespace hdr_attr_ctrl
