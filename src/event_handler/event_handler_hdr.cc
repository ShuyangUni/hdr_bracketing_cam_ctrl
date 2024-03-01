/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-24 15:35:34
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-25 17:17:09
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/event_handler/event_handler_hdr.h"

namespace hdr_attr_ctrl {
EventHandlerHDR::EventHandlerHDR(const ros::NodeHandle &nh,
                                 const Spinnaker::CameraPtr &cam,
                                 const std::shared_ptr<ContextHDR> &context)
    : EventHandler(nh, cam) {
  context_ = context;
  CameraResponseFunction crf(context_->hdr_g_func_filepath.c_str());
  seed_sampler_ = std::shared_ptr<FunctionSeedSampler>(new FunctionSeedSampler(
      crf, context_->ss_n_seed, context_->ss_range_ratio,
      context_->ss_sigma_ratio, context_->ss_gap_expo_val_min,
      context_->ss_gap_expo_val_max, context_->hdr_resize_h,
      context_->hdr_resize_w));
  gpr_ = std::shared_ptr<GPRegressor>(
      new GPRegressor(context_->gpr_hyper_c, context_->gpr_px_sample_res));
  InitCtrl();
  expo_major_.expo_t = context_->hdr_expo_init_major.expo_t;
  expo_major_.gain = context_->hdr_expo_init_major.gain;
  expo_low_.expo_t = context_->hdr_expo_init_low.expo_t;
  expo_low_.gain = context_->hdr_expo_init_low.gain;
  expo_high_.expo_t = context_->hdr_expo_init_high.expo_t;
  expo_high_.gain = context_->hdr_expo_init_high.gain;
  expo_val_tar_major_ = Exposure2ExposureVal(expo_major_);
  score_tar_major_ = 0.0;

  double drift = context_->hdr_expo_t_shift;
  range_expo_t_major_.first = context_->range_expo_t_major.first + drift;
  range_expo_t_major_.second = context_->range_expo_t_major.second - drift;
  range_gain_major_ = context_->range_gain_major;
  range_expo_t_low_.first = context_->range_expo_t_major.first;
  range_expo_t_low_.second = context_->range_expo_t_major.second - drift * 2;
  range_gain_low_ = context_->range_gain_major;
  range_expo_t_high_.first = context_->range_expo_t_major.first + drift * 2;
  range_expo_t_high_.second = context_->range_expo_t_major.second;
  range_gain_high_ = context_->range_gain_major;

  pub_image_raw_ = nh_.advertise<sensor_msgs::Image>(
      context_->ros_topic_name_image_raw.c_str(), 10);
  std::string str_topic_full = context_->ros_topic_name_image_raw + "_full";
  pub_image_full_ =
      nh_.advertise<sensor_msgs::Image>(str_topic_full.c_str(), 10);
}

void EventHandlerHDR::OnImageEvent(Spinnaker::ImagePtr image) {
  ts_ = ros::Time::now();
  std_msgs::Header header;
  header.stamp = ts_;
  sensor_msgs::ImagePtr img_msg;
  Exposure expo;
  // get image
  try {
    // check whether image is complete
    if (image->IsIncomplete()) {
      ROS_WARN("Image incomplete with image status %d !",
               image->GetImageStatus());
      return;
    }
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
    // get image & resize
    Convert2Mat(image, &img_cur_);
    if (img_cur_.type() == CV_8UC1) {
      // mono8
      img_msg = cv_bridge::CvImage(header, "mono8", img_cur_).toImageMsg();
    } else {
      // bgr8
      img_msg = cv_bridge::CvImage(header, "bgr8", img_cur_).toImageMsg();
    }
  } catch (Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
  ROS_INFO("get image: [%.1f, %.1f]", expo_cur_.expo_t, expo_cur_.gain);

  /**
   * core process
   */
  // update seeds
  seed_sampler_->Process(img_cur_, expo, &seeds_cur_);
  ExpoState expo_state = FindExpoState(expo_cur_);
  if (context_->publish_all_images) {
    pub_image_raw_.publish(img_msg);
  } else {
    if (expo_state == k_expo_major_) pub_image_raw_.publish(img_msg);
    pub_image_full_.publish(img_msg);
  }
  ++seq_id_;
  // ROS_INFO("expo_state: %d", expo_state);
  if (!UpdateSeedsByExpoState(expo_state, expo_cur_, seeds_cur_)) return;
  // update camera attribute
  FuncSamples seeds_train, seeds_predict;
  if (!AssembleSeeds(expo_state, &seeds_train)) return;
  gpr_->GPRInference(seeds_train, &seeds_predict);
  // get exposure value
  double expo_val_major, score_major;
  FindBestExposure(seeds_predict, &expo_val_major, &score_major);
  ROS_INFO("current/target optimal expo_val: %.2f / %.2f", expo_val_major,
           expo_val_tar_major_);
  ROS_INFO("current/target optimal score: %.2f / %.2f", score_major,
           score_tar_major_);
  // check whether update
  if (!JudgeWhetherUpdate(expo_val_major, score_major)) return;
  // update
  double expo_val_low = expo_val_major * context_->hdr_expo_val_ratio_low;
  double expo_val_high = expo_val_major * context_->hdr_expo_val_ratio_high;
  expo_val_tar_major_ = expo_val_major;
  score_tar_major_ = score_major;
  // get exposure
  Exposure expo_major, expo_low, expo_high;
  expo_major = ExposureVal2Exposure_ExposureFirst(
      expo_val_major, range_expo_t_major_.second + context_->expo_val_offset);
  expo_low = ExposureVal2Exposure_ExposureFirst(
      expo_val_low, range_expo_t_low_.second + context_->expo_val_offset);
  expo_high = ExposureVal2Exposure_ExposureFirst(
      expo_val_high, range_expo_t_high_.second + context_->expo_val_offset);
  expo_major = CheckExposure(
      expo_major, range_expo_t_major_.second + context_->expo_val_offset,
      range_expo_t_major_.first + context_->expo_val_offset,
      range_gain_major_.second, range_gain_major_.first);
  expo_low = CheckExposure(expo_low,
                           range_expo_t_low_.second + context_->expo_val_offset,
                           range_expo_t_low_.first + context_->expo_val_offset,
                           range_gain_low_.second, range_gain_low_.first);
  expo_high = CheckExposure(
      expo_high, range_expo_t_high_.second + context_->expo_val_offset,
      range_expo_t_high_.first + context_->expo_val_offset,
      range_gain_high_.second, range_gain_high_.first);
  expo_major.expo_t -= context_->expo_val_offset;
  expo_low.expo_t -= context_->expo_val_offset;
  expo_high.expo_t -= context_->expo_val_offset;
  HandleAmbiguity(&expo_major, range_expo_t_major_);
  HandleAmbiguity(&expo_low, range_expo_t_low_);
  HandleAmbiguity(&expo_high, range_expo_t_high_);
  expo_major_ = expo_major;
  expo_low_ = expo_low;
  expo_high_ = expo_high;
  context_->hdr_mutex.lock();
  UpdateSequenceStates(&context_->hdr_sequencer_states);
  context_->hdr_b_update = true;
  context_->hdr_mutex.unlock();
  InitCtrl();
  ROS_WARN("target expo major: [%.1f, %.1f]", expo_major_.expo_t,
           expo_major_.gain);
  ROS_WARN("target expo low: [%.1f, %.1f]", expo_low_.expo_t, expo_low_.gain);
  ROS_WARN("target expo high: [%.1f, %.1f]", expo_high_.expo_t,
           expo_high_.gain);
}

ExpoState EventHandlerHDR::FindExpoState(const Exposure &expo) {
  ExpoState expo_state_out = k_expo_idle_;
  if (CheckEquality(expo, expo_major_)) expo_state_out += k_expo_major_;
  if (CheckEquality(expo, expo_low_)) expo_state_out += k_expo_low_;
  if (CheckEquality(expo, expo_high_)) expo_state_out += k_expo_high_;
  return expo_state_out;
}

void EventHandlerHDR::InitCtrl() {
  valid_major_ = false;
  valid_low_ = false;
  valid_high_ = false;
}

bool EventHandlerHDR::UpdateSeedsByExpoState(const ExpoState &expo_state,
                                             const Exposure &expo_cur,
                                             const FuncSamples &seeds_cur) {
  bool state = false;
  if (expo_state & k_expo_major_) {
    expo_major_ = expo_cur;
    seeds_major_.assign(seeds_cur.begin(), seeds_cur.end());
    valid_major_ = true;
    state = true;
  }
  if (expo_state & k_expo_low_) {
    expo_low_ = expo_cur;
    seeds_low_.assign(seeds_cur.begin(), seeds_cur.end());
    valid_low_ = true;
    state = true;
  }
  if (expo_state & k_expo_high_) {
    expo_high_ = expo_cur;
    seeds_high_.assign(seeds_cur.begin(), seeds_cur.end());
    valid_high_ = true;
    state = true;
  }
  return state;
}

bool EventHandlerHDR::AssembleSeeds(const ExpoState &expo_state,
                                    FuncSamples *seeds_train) {
  bool state = false;
  seeds_train->clear();
  switch (expo_state) {
    case k_expo_major_:
    case k_expo_low_:
    case k_expo_high_:
      if (valid_major_ && valid_low_ && valid_high_) {
        seeds_train->assign(seeds_low_.begin(), seeds_low_.end());
        seeds_train->insert(seeds_train->end(), seeds_major_.begin(),
                            seeds_major_.end());
        seeds_train->insert(seeds_train->end(), seeds_high_.begin(),
                            seeds_high_.end());
        state = true;
      }
      break;
    case k_expo_major_ + k_expo_low_:
      if (valid_major_ && valid_high_) {
        seeds_train->assign(seeds_major_.begin(), seeds_major_.end());
        seeds_train->insert(seeds_train->end(), seeds_high_.begin(),
                            seeds_high_.end());
        state = true;
      }
      break;
    case k_expo_major_ + k_expo_high_:
      if (valid_major_ && valid_low_) {
        seeds_train->assign(seeds_low_.begin(), seeds_low_.end());
        seeds_train->insert(seeds_train->end(), seeds_major_.begin(),
                            seeds_major_.end());
        state = true;
      }
      break;
    default:
      ROS_INFO("unexpected exposure state is found...state: %d", expo_state);
      break;
  }
  return state;
}

void EventHandlerHDR::FindBestExposure(const FuncSamples &seeds_predict,
                                       double *expo_val_opt,
                                       double *score_opt) {
  double score_max = -std::numeric_limits<double>::infinity();
  double expo_val_max = -1.0;
  int idx_max = 0;
  for (size_t i = 0; i < seeds_predict.size(); ++i) {
    if (seeds_predict.at(i).score > score_max) {
      score_max = seeds_predict.at(i).score;
      expo_val_max = seeds_predict.at(i).expo_val;
      idx_max = i;
    }
  }
  int range = 100;
  double ratio = 0.9;
  int idx_range_left = idx_max - range < 0 ? 0 : idx_max - range;
  int idx_range_right = idx_max + range > seeds_predict.size()
                            ? seeds_predict.size()
                            : idx_max + range;
  std::vector<double> expo_val_select;
  std::vector<double> weight_select;
  for (size_t i = idx_range_left; i < idx_range_right; ++i) {
    if (seeds_predict.at(i).score > score_max * ratio) {
      double weight = (seeds_predict.at(i).score - score_max * ratio) /
                      (score_max * (1.0 - ratio));
      expo_val_select.push_back(seeds_predict.at(i).expo_val);
      weight_select.push_back(weight);
    }
  }
  double sum_weight = 0.0;
  double sum_expo_val = 0.0;
  double sum_score = 0.0;
  for (size_t i = 0; i < expo_val_select.size(); ++i) {
    sum_weight += weight_select.at(i);
    sum_expo_val += expo_val_select.at(i) * weight_select.at(i);
  }
  *expo_val_opt = sum_expo_val / sum_weight;
  *score_opt = score_max;
}

bool EventHandlerHDR::JudgeWhetherUpdate(const double &expo_val_cur,
                                         const double &score_cur) {
  bool b_update = false;
  if (fabs(score_tar_major_ - score_cur) / score_cur >
      context_->hdr_ctrl_score_diff_ratio) {
    b_update = true;
  } else {
    if (fabs(expo_val_tar_major_ - expo_val_cur) >
            context_->hdr_ctrl_expo_val_diff_abs &&
        fabs(expo_val_tar_major_ - expo_val_cur) / expo_val_cur >
            context_->hdr_ctrl_expo_val_diff_ratio) {
      b_update = true;
    } else {
      b_update = false;
    }
  }
  return b_update;
}

void EventHandlerHDR::UpdateSequenceStates(SequenceStates *states) {
  states->clear();
  SequenceState state;
  state.expo = expo_major_;
  state.seq_id = 0;
  states->push_back(state);
  state.expo = expo_low_;
  state.seq_id = 1;
  states->push_back(state);
  state.expo = expo_major_;
  state.seq_id = 2;
  states->push_back(state);
  state.expo = expo_high_;
  state.seq_id = 3;
  states->push_back(state);
}

void EventHandlerHDR::HandleAmbiguity(Exposure *expo,
                                      const Range &range_expo_t) {
  if (CheckEquality(*expo, expo_major_) || CheckEquality(*expo, expo_low_) ||
      CheckEquality(*expo, expo_high_)) {
    if (fabs(range_expo_t.first - expo->expo_t) >
        context_->hdr_expo_t_shift / 2) {
      expo->expo_t -= context_->hdr_expo_t_shift / 2;
    } else {
      expo->expo_t += context_->hdr_expo_t_shift / 2;
    }
  }
}

}  // namespace hdr_attr_ctrl
