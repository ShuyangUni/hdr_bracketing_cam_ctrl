/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-24 15:35:34
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-25 16:51:54
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/event_handler/event_handler_kim.h"

namespace hdr_attr_ctrl {
EventHandlerKim::EventHandlerKim(const ros::NodeHandle &nh,
                                 const Spinnaker::CameraPtr &cam,
                                 const std::shared_ptr<ContextKim> &context)
    : EventHandler(nh, cam) {
  context_ = context;
  // init
  expo_tar_.expo_t = context_->kim_seed_expo_t;
  expo_tar_.gain = context_->kim_seed_gain;
  b_wait_seed_ = true;
  b_first_after_seed_ = false;
  crf_ = std::shared_ptr<CameraResponseFunction>(
      new CameraResponseFunction(context_->kim_g_func_filepath.c_str()));
  gpr_ = std::shared_ptr<GPRegressor>(new GPRegressor(
      context_->kim_gp_hyper_c, context_->kim_gp_hyper_l, 0.01));
  GeneratePx();
  metric_opt_ = 0.0;
  metric_cur_ = 0.0;
  // register camera topics
  pub_image_raw_ = nh_.advertise<sensor_msgs::Image>(
      context_->ros_topic_name_image_raw.c_str(), 10);
}

void EventHandlerKim::OnImageEvent(Spinnaker::ImagePtr image) {
  ts_ = ros::Time::now();
  std_msgs::Header header;
  header.stamp = ts_;
  sensor_msgs::ImagePtr img_msg;
  Spinnaker::GenApi::INodeMap &node_map = cam_->GetNodeMap();
  Exposure expo;
  try {
    // check whether image is complete
    if (image->IsIncomplete()) {
      ROS_WARN("Image incomplete with image status %d !",
               image->GetImageStatus());
      return;
    }
    // get image & resize
    Convert2Mat(image, &img_cv_);
    cv::Mat img_cv;
    img_cv = img_cv_.clone();
    // get attributes
    Spinnaker::ChunkData chunkdata = image->GetChunkData();
    expo_cur_.expo_t = static_cast<double>(chunkdata.GetExposureTime());
    expo_cur_.gain = static_cast<double>(chunkdata.GetGain());
    expo = expo_cur_;
    expo.expo_t += context_->expo_val_offset;
    char attr_msg_char[200];
    snprintf(attr_msg_char, sizeof(attr_msg_char),
             "seq: %u expo: %.0f gain: %.1f", seq_id_, expo.expo_t,
             expo.gain);
    header.frame_id = attr_msg_char;
    // get image
    if (img_cv.type() == CV_8UC1) {
      // mono8
      img_msg = cv_bridge::CvImage(header, "mono8", img_cv).toImageMsg();
    } else {
      // bgr8
      img_msg = cv_bridge::CvImage(header, "bgr8", img_cv).toImageMsg();
    }
  } catch (Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }

  // get current exposure value
  double expo_val_cur = Exposure2ExposureVal(expo);

  // ROS_INFO("expo_cur_: <%.1f, %.1f>, expo_tar_: <%.1f, %.1f>",
  // expo_cur_.expo_t,
  //          expo_cur_.gain, expo_tar_.expo_t, expo_tar_.gain);

  // check whether get target exposure
  if (CheckEquality(expo_cur_, expo_tar_)) {
    // check whether target exposure is seed
    if (b_wait_seed_) {
      // for seed image
      // double t0 = ros::Time::now().toSec();
      double expo_val_next = ProcessSeed(img_cv_, expo_val_cur);
      // update exposure
      expo_tar_ = ExposureVal2Exposure_ExposureFirst(
          expo_val_next,
          context_->range_expo_t_major.second + context_->expo_val_offset);
      expo_tar_.expo_t -= context_->expo_val_offset;
      ChangeExposureTime(node_map, expo_tar_.expo_t);
      ChangeGain(node_map, expo_tar_.gain);
      b_wait_seed_ = false;
      b_first_after_seed_ = true;
      ROS_WARN("change exposure to <%.1f, %.1f>", expo_tar_.expo_t,
               expo_tar_.gain);
      // double t1 = ros::Time::now().toSec();
      // ROS_INFO("seed image process: %fms", (t1 - t0) * 1000);
    } else {
      // double t0 = ros::Time::now().toSec();
      // for common image
      // publish
      if (context_->publish_all_images) {
        pub_image_raw_.publish(img_msg);
      } else {
        if (b_first_after_seed_) {
          pub_image_raw_.publish(img_msg);
          b_first_after_seed_ = false;
        } else {
          if (seq_id_ % 2 == 0) pub_image_raw_.publish(img_msg);
        }
      }
      ++seq_id_;
      // calc current metric
      cv::Mat img_resize;
      cv::resize(
          img_cv_, img_resize,
          cv::Size(context_->kim_img_resize_w, context_->kim_img_resize_h));
      metric_cur_ = CalcMetricKim(img_resize);
      // check global illumination
      if (NeedChangeExposure()) {
        expo_tar_.expo_t = context_->kim_seed_expo_t;
        expo_tar_.gain = context_->kim_seed_gain;
        ChangeExposureTime(node_map, expo_tar_.expo_t);
        ChangeGain(node_map, expo_tar_.gain);
        b_wait_seed_ = true;
      }
      // double t1 = ros::Time::now().toSec();
      // ROS_INFO("common image check: %fms", (t1 - t0) * 1000);
    }
  }
}

bool EventHandlerKim::NeedChangeExposure() {
  bool b_need_change = true;
  // ROS_INFO("metric_opt_: %.3f, metric_cur_: %.3f", metric_opt_, metric_cur_);
  // if ((metric_cur_ / metric_opt_) > 0.7 && (metric_cur_ / metric_opt_) < 1.4) {
  //   b_need_change = false;
  // }
  if ((metric_cur_ / metric_opt_) > 0.8 && (metric_cur_ / metric_opt_) < 1.2) {
    b_need_change = false;
  }
  return b_need_change;
}

double EventHandlerKim::ProcessSeed(const cv::Mat &img_seed,
                                    const double &expo_val_seed) {
  // get irradiance image
  Eigen::MatrixXf irr_eigen;
  crf_->Image2Irradiance(img_seed, expo_val_seed, &irr_eigen);
  // init
  psi_ = 0.0;
  // bayesian optimization
  Eigen::VectorXd tx, ty;
  std::vector<double> tx_vec, ty_vec;
  cv::Mat img_resize, img_synthetic;
  Eigen::VectorXd py, pvar;
  double expo_next = expo_val_seed;
  for (size_t i = 0; i < context_->kim_bo_max_iter; ++i) {
    if (i == 0) {
      // first frame, seed
      cv::resize(
          img_seed, img_resize,
          cv::Size(context_->kim_img_resize_w, context_->kim_img_resize_h));
    } else {
      // other frame, synthetic
      crf_->Irradiance2Image(irr_eigen, expo_next, &img_synthetic);
      cv::resize(
          img_synthetic, img_resize,
          cv::Size(context_->kim_img_resize_w, context_->kim_img_resize_h));
    }
    double metric = CalcMetricKim(img_resize);
    // ROS_INFO("seed %lu: query_point: %.1f, metric: %.1f", i, expo_next,
    // metric);
    tx_vec.push_back(expo_next);
    ty_vec.push_back(metric);
    tx = Eigen::Map<Eigen::VectorXd>(tx_vec.data(), tx_vec.size());
    ty = Eigen::Map<Eigen::VectorXd>(ty_vec.data(), ty_vec.size());
    GPPredict(tx, ty, px_, &py, &pvar);
    expo_next = FindQueryPoint(px_, py, pvar);
  }
  // find best exposure val
  int index;
  double cost = py.maxCoeff(&index);
  double expo_val_next = px_(index);
  // calc best metric
  crf_->Irradiance2Image(irr_eigen, expo_val_next, &img_synthetic);
  cv::resize(img_synthetic, img_resize,
             cv::Size(context_->kim_img_resize_w, context_->kim_img_resize_h));
  metric_opt_ = CalcMetricKim(img_resize);
  // ROS_INFO("optimal metric: %.1f", metric_opt_);
  return expo_val_next;
}

void EventHandlerKim::GPPredict(const Eigen::VectorXd &tx,
                                const Eigen::VectorXd &ty,
                                const Eigen::VectorXd &px, Eigen::VectorXd *py,
                                Eigen::VectorXd *pvar) {
  Eigen::MatrixXd kernel;
  gpr_->CalcKernel(tx, tx, &kernel);
  kernel += context_->kim_gp_hyper_sn * context_->kim_gp_hyper_sn *
            Eigen::MatrixXd::Identity(tx.size(), tx.size());
  Eigen::MatrixXd invkernel = kernel.llt().solve(
      Eigen::MatrixXd::Identity(kernel.rows(), kernel.cols()));
  Eigen::MatrixXd invkernel_ty = invkernel * ty;
  Eigen::MatrixXd kernel_s;
  double kernel_ss;
  *py = Eigen::VectorXd::Zero(px.size());
  *pvar = Eigen::VectorXd::Zero(px.size());
  for (size_t i = 0; i < px.size(); ++i) {
    gpr_->CalcKernel(px_(i), tx, &kernel_s);
    gpr_->CalcKernel(px_(i), px_(i), &kernel_ss);
    (*py)(i) = (kernel_s * invkernel_ty).value();
    (*pvar)(i) =
        kernel_ss - (kernel_s * invkernel * kernel_s.transpose()).value();
  }
}

double EventHandlerKim::FindQueryPoint(const Eigen::VectorXd &px,
                                       const Eigen::VectorXd &py,
                                       const Eigen::VectorXd &pvar) {
  int n_pred = px.size();
  Eigen::VectorXd var_diag_sq = pvar.cwiseProduct(pvar);
  Eigen::VectorXd tmp = psi_ * Eigen::VectorXd::Ones(n_pred);
  var_diag_sq = var_diag_sq + tmp;
  tmp = sqrt(psi_) * Eigen::VectorXd::Ones(n_pred);
  var_diag_sq = var_diag_sq.cwiseSqrt() - tmp;
  var_diag_sq = sqrt(context_->kim_bo_alpha) * var_diag_sq;
  Eigen::VectorXd acq_func = py + var_diag_sq;
  int index;
  double cost = acq_func.maxCoeff(&index);
  psi_ = psi_ + pvar(index);
  return px(index);
}

void EventHandlerKim::GeneratePx() {
  std::vector<double> px_vec;
  for (double expo_t = context_->range_expo_t_major.first;
       expo_t < context_->range_expo_t_major.second;
       expo_t += context_->kim_gp_interval_expo_t) {
    for (double gain = context_->range_gain_major.first;
         gain < context_->range_gain_major.second;
         gain += context_->kim_gp_interval_gain) {
      Exposure expo;
      expo.expo_t = expo_t + context_->expo_val_offset;
      expo.gain = gain;
      double expo_val = Exposure2ExposureVal(expo);
      px_vec.push_back(expo_val);
    }
  }
  // std::vector to Eigen::VectorXd
  px_ = Eigen::Map<Eigen::VectorXd>(px_vec.data(), px_vec.size());
}

double EventHandlerKim::CalcMetricKim(const cv::Mat &img) {
  // from Kim's code
  // cv::Mat entropy;
  // CalcImageLocalEntropy(img, &entropy);
  // cv::Mat wmask;
  // CalcWeightMask(entropy, &wmask);
  // cv::Mat grad;
  // CalcGrad(img, &grad);
  // double gmean = sum(grad)[0] / (grad.rows * grad.cols);
  // double emean = sum(entropy)[0] / (entropy.rows * entropy.cols);
  // cv::Mat gradw = grad > gmean * 0.01;
  // gradw *= 1;
  // gradw.convertTo(gradw, CV_32F, 1.0 / 255.0);
  // double satparam = -4.8;
  // cv::Mat smask = satparam * gmean * wmask;
  // cv::Mat gour = ((gradw.mul(grad)) + smask);
  // return CalcImgGours(gour);

  // average gradient magnitude only, for test
  // return cv::mean(CalcImageGradientMagnitude(img))[0];

  // kim2020tro
  cv::Mat gm = CalcImageGradientMagnitude(img);
  cv::Mat entropy;
  CalcImageLocalEntropy(img, &entropy);
  cv::Scalar e_mean, e_std;
  cv::meanStdDev(entropy, e_mean, e_std);
  // calc weight
  cv::Mat weight;
  cv::pow(entropy - e_mean[0], 2, weight);
  weight = weight / (-2 * e_std[0] * e_std[0]);
  cv::exp(weight, weight);
  cv::normalize(weight, weight, 0, 1, cv::NORM_MINMAX);
  // calc pi
  double alpha = 64.0;
  double tau = 4.0;
  cv::Mat pi;
  pi = entropy * (-alpha) + tau;
  cv::exp(pi, pi);
  pi = 2.0 / (pi + 1.0) - 1.0;
  // calc mask
  cv::Mat mask;
  CalcWeightMask(entropy, &mask);
  // calc u
  gm.convertTo(gm, CV_32F, 1.0);
  weight.convertTo(weight, CV_32F, 1.0);
  pi.convertTo(pi, CV_32F, 1.0);
  mask.convertTo(mask, CV_32F, 1.0);
  cv::Mat u = weight.mul(gm);
  u = u + (pi.mul(mask)).mul(weight) * cv::mean(gm)[0];
  return cv::mean(u)[0];
}

void EventHandlerKim::CalcWeightMask(const cv::Mat &entropy, cv::Mat *wmask) {
  // Create Entropy binary mask, wmask == [rs, cs]
  cv::Mat wmasktmp(entropy.size(), CV_32F, 0.);
  cv::Mat tmp;
  wmasktmp = entropy <= 0.01;
  wmasktmp.convertTo(tmp, CV_32F, 1.0 / 255.0);
  *wmask = tmp;
}

void EventHandlerKim::CalcGrad(const cv::Mat &img, cv::Mat *grad) {
  cv::Mat Gx, Gy;
  double ksize = 1;
  cv::Mat abs_grad_x, abs_grad_y;
  cv::Sobel(img, Gx, CV_8U, 1, 0, ksize);
  convertScaleAbs(Gx, abs_grad_x);
  cv::Sobel(img, Gy, CV_8U, 0, 1, ksize);
  convertScaleAbs(Gy, abs_grad_y);
  addWeighted(abs_grad_x, 1.49999, abs_grad_y, 1.49999, 0, *grad);
  grad->convertTo(*grad, CV_32F, 1.0 / 255.0);
}

double EventHandlerKim::CalcImgGours(const cv::Mat &gour) {
  cv::Mat gours_tmp1, gours_tmp2;
  for (int i = 0; i < gour.rows; i++)
    gours_tmp1.push_back(cv::sum(gour.row(i))[0]);
  for (int j = 0; j < gours_tmp1.cols; j++)
    gours_tmp2.push_back(cv::sum(gours_tmp1.col(j))[0]);
  return gours_tmp2.at<double>(0);
}

}  // namespace hdr_attr_ctrl
