/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-28 22:32:40
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-29 01:08:01
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <immintrin.h>

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace hdr_attr_ctrl {
template <unsigned int N>
class Polynomial {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void Fitting(const std::vector<double> &x, const std::vector<double> &y) {
    assert(x.size() != 0);
    assert(y.size() != 0);
    assert(x.size() == y.size());
    // prepare least square
    const Eigen::VectorXd vec_b =
        Eigen::Map<const Eigen::VectorXd, Eigen::Aligned>(y.data(), y.size());
    Eigen::MatrixXd mat_a = Eigen::MatrixXd::Zero(x.size(), N + 1);
    double value = 0.0;
    for (size_t i = 0; i < x.size(); ++i) {
      value = 1.0;
      for (size_t j = 0; j < N + 1; ++j) {
        mat_a(i, j) = value;
        value *= x.at(i);
      }
    }
    // solve least square
    // direct method, fast but may be singular
    Eigen::VectorXd vec_res =
        (mat_a.transpose() * mat_a).ldlt().solve(mat_a.transpose() * vec_b);
    std::copy(vec_res.data(), vec_res.data() + vec_res.size(), &coeffient_[0]);
  }

  void Inference(const double &x, double *y) {
    *y = 0.0;
    double value = 1.0;
    for (size_t j = 0; j < N + 1; ++j) {
      *y += value * coeffient_[j];
      value = value * x;
    }
  }

  void Inference(const std::vector<double> &x, std::vector<double> *y) {
    y->clear();
    y->resize(x.size());
    double value = 0.0;
    for (size_t i = 0; i < x.size(); ++i) {
      value = 1.0;
      y->at(i) = 0.0;
      for (size_t j = 0; j < N + 1; ++j) {
        y->at(i) += value * coeffient_[j];
        value = value * x.at(i);
      }
    }
  }

  void Inference(const Eigen::MatrixXf &x, Eigen::MatrixXf *y) {
    *y = Eigen::MatrixXf::Zero(x.rows(), x.cols());
    Eigen::MatrixXf value = Eigen::MatrixXf::Ones(x.rows(), x.cols());
    // for (size_t j = 0; j < 11; ++j) {
    //   y->array() += value.array() * param.param_array[j];
    //   value.array() *= x.array();
    // }
    __m256 p[N + 1];
    for (size_t i = 0; i < N + 1; ++i)
      for (size_t j = 0; j < 8; ++j)
        ((float *)&p[i])[j] = coeffient_[i];  // NOLINT
    __m256 *yyy = reinterpret_cast<__m256 *>(&y->data()[0]);
    __m256 *vvv = reinterpret_cast<__m256 *>(&value.data()[0]);
    const __m256 *xxx = reinterpret_cast<const __m256 *>(&x.data()[0]);
    __m256 *end_yyy = reinterpret_cast<__m256 *>(&y->data()[y->size()]);
    for (; yyy != end_yyy; vvv++, yyy++, xxx++) {
      for (size_t i = 0; i < N + 1; ++i) {
        // *yyy = _mm256_add_ps(*yyy, _mm256_mul_ps(*vvv, p[i]));
        *yyy = _mm256_fmadd_ps(*vvv, p[i], *yyy);
        *vvv = _mm256_mul_ps(*vvv, *xxx);
      }
    }
  }

 private:
  double coeffient_[N + 1];
};
}  // namespace hdr_attr_ctrl
