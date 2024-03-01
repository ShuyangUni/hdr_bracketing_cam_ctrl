/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-26 16:00:20
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-08 16:44:10
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once
#include <Eigen/Dense>

#include "include/common.h"

namespace hdr_attr_ctrl {
class GPRegressor {
 public:
  GPRegressor(const double &hyper_c, const double &px_sample_res);
  GPRegressor(const double &hyper_c, const double &hyper_l,
              const double &px_sample_res);
  void GPRInference(const FuncSamples &data_train, FuncSamples *data_predict);

  void CalcKernel(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2,
                  Eigen::MatrixXd *kernel);
  void CalcKernel(const Eigen::VectorXd &x1, const double &x2,
                  Eigen::MatrixXd *kernel);
  void CalcKernel(const double &x1, const Eigen::VectorXd &x2,
                  Eigen::MatrixXd *kernel);
  void CalcKernel(const double &x1, const double &x2, double *kernel);

  void Preprocess(const FuncSamples &data_train);
  void Postprocess(FuncSamples *data_predict);
  void NormalizeY();
  double CheckDoubleZero(const double &val);

 private:
  // hyper parameter
  double hyper_c2_;
  double hyper_l2_;
  double hyper_eta_;
  double px_sample_res_;

  // data
  Eigen::MatrixXd kernel_;
  Eigen::VectorXd tx_;
  Eigen::VectorXd ty_;
  Eigen::VectorXd tsigma_;
  Eigen::VectorXd px_;
  Eigen::VectorXd py_;
  Eigen::VectorXd psigma_;

  double ty_mean_;
  double ty_std_;
  Eigen::MatrixXd invkernel_;
  Eigen::MatrixXd invkernel_ny_;
};
}  // namespace hdr_attr_ctrl
