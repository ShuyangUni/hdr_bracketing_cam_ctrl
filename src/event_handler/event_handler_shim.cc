/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-24 15:35:34
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-09 16:11:09
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/event_handler/event_handler_shim.h"

namespace hdr_attr_ctrl {
EventHandlerShim::EventHandlerShim(const ros::NodeHandle &nh,
                                   const Spinnaker::CameraPtr &cam,
                                   const std::shared_ptr<ContextShim> &context)
    : EventHandler(nh, cam) {
  context_ = context;
  // generate seeds
  gamma_seeds_ = {0.1, 0.5, 0.8, 1.0, 1.2, 1.5, 1.9};
  gamma_samples_.clear();
  for (double gamma = 0.1; gamma <= 1.9;
       gamma = gamma + context_->shim_sample_res)
    gamma_samples_.push_back(gamma);
  // generate luts
  lut_list_.clear();
  for (size_t i = 0; i < gamma_seeds_.size(); ++i) {
    cv::Mat lut;
    GenerateGammaCorrectionLUT(&lut, gamma_seeds_.at(i));
    lut_list_.push_back(lut);
  }
  // register camera topics
  pub_image_raw_ = nh_.advertise<sensor_msgs::Image>(
      context_->ros_topic_name_image_raw.c_str(), 10);
}

void EventHandlerShim::GenerateGammaCorrectionLUT(cv::Mat *lut,
                                                  const double &gamma) {
  double inv_gamma = 1.0 / gamma;
  lut->create(1, 256, CV_8UC1);
  uchar *p = lut->data;
  for (int i = 0; i < 256; ++i) p[i] = pow(i / 255.0, inv_gamma) * 255;
}

double EventHandlerShim::ExposureUpdate(const double &expo_val_cur,
                                        const double &gamma) {
  double alpha = context_->shim_alpha_neg;
  if (gamma >= 1.0) alpha = context_->shim_alpha_pos;
  return expo_val_cur * (1.0 + alpha * context_->shim_k_p * (gamma - 1.0));
}

void EventHandlerShim::OnImageEvent(Spinnaker::ImagePtr image) {
  ts_ = ros::Time::now();
  std_msgs::Header header;
  header.stamp = ts_;
  sensor_msgs::ImagePtr img_msg;
  Spinnaker::GenApi::INodeMap &node_map = cam_->GetNodeMap();
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
    expo_cur_.expo_t += context_->expo_val_offset;
    char attr_msg_char[200];
    snprintf(attr_msg_char, sizeof(attr_msg_char),
             "seq: %u expo: %.0f gain: %.1f", seq_id_, expo_cur_.expo_t,
             expo_cur_.gain);
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
  // publish
  if (context_->publish_all_images) {
    pub_image_raw_.publish(img_msg);
  } else {
    if (seq_id_ % 2 == 0) pub_image_raw_.publish(img_msg);
  }
  ++seq_id_;

  // control process
  // generate fitting data
  std::vector<double> gm_seeds;
  cv::Mat img_gamma;
  for (size_t i = 0; i < gamma_seeds_.size(); ++i) {
    cv::LUT(img_cv_, lut_list_.at(i), img_gamma);
    gm_seeds.push_back(CalcImageGradientMagnitudeSum(img_gamma));
  }
  // fitting curve
  Polynomial<5> polynomial;
  polynomial.Fitting(gamma_seeds_, gm_seeds);
  // get curve values of gamma samples
  std::vector<double> gm_samples;
  polynomial.Inference(gamma_samples_, &gm_samples);
  // get maximum value
  int idx_max = std::max_element(gm_samples.begin(), gm_samples.end()) -
                gm_samples.begin();
  double gamma_optimal = gamma_samples_.at(idx_max);
  // update attributes
  double expo_val_cur = Exposure2ExposureVal(expo_cur_);
  double expo_val_tar = ExposureUpdate(expo_val_cur, gamma_optimal);
  Exposure expo_tar = ExposureVal2Exposure_ExposureFirst(
      expo_val_tar,
      context_->range_expo_t_major.second + context_->expo_val_offset);
  expo_tar.expo_t -= context_->expo_val_offset;
  expo_tar = CheckExposure(expo_tar, context_->range_expo_t_major.second,
                           context_->range_expo_t_major.first,
                           context_->range_gain_major.second,
                           context_->range_gain_major.first);
  // ctrl attributes
  ChangeExposureTime(node_map, expo_tar.expo_t);
  ChangeGain(node_map, expo_tar.gain);
  // ROS_INFO("processed imgae seq_id: %u", seq_id_);
}

}  // namespace hdr_attr_ctrl
