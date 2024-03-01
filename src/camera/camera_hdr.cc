/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-01 23:10:20
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-28 23:36:56
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/camera/camera_hdr.h"

namespace hdr_attr_ctrl {
CameraHDR::CameraHDR(const Spinnaker::CameraPtr &cam_ptr,
                     const ros::NodeHandle &nh, const size_t &global_idx) {
  nh_ = nh;
  cam_ = cam_ptr;
  context_ =
      std::dynamic_pointer_cast<ContextHDR>(g_context_list.at(global_idx));
}

void CameraHDR::Run() {
  InitCamera(cam_, context_->cam_serial);
  Spinnaker::GenApi::INodeMap &node_map = cam_->GetNodeMap();
  InitCameraForSequencer(node_map);
  RegisterController();
  // init states
  SequenceStates states;
  InitSequencerStates(&states);
  ConfigureSequencer(node_map, states);
  ros::Duration(0.1).sleep();
  BeginCapture(cam_, context_->cam_serial);

  SequenceStates sequencer_states;
  ros::Rate r(30);
  while (ros::ok()) {
    // whether update sequencer
    context_->hdr_mutex.lock();
    if (context_->hdr_b_update) {
      sequencer_states.assign(context_->hdr_sequencer_states.begin(),
                              context_->hdr_sequencer_states.end());
      context_->hdr_b_update = false;
      context_->hdr_mutex.unlock();
      // update sequencer states
      EndCapture(cam_, context_->cam_serial);
      ConfigureSequencer(node_map, sequencer_states);
      BeginCapture(cam_, context_->cam_serial);
      ROS_INFO("finish update sequencer states...");
    } else {
      context_->hdr_mutex.unlock();
    }
    r.sleep();
  }
}

void CameraHDR::RegisterController() {
  try {
    controller_ = std::shared_ptr<EventHandlerHDR>(
        new EventHandlerHDR(nh_, cam_, context_));
    cam_->RegisterEventHandler(*controller_);
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

void CameraHDR::InitSequencerStates(SequenceStates *states) {
  states->clear();
  SequenceState state;
  // state 0
  state.seq_id = 0;
  state.expo.gain = 0.0;
  state.expo.expo_t = 4000;
  states->push_back(state);
  // state 1
  state.seq_id = 1;
  state.expo.gain = 0.0;
  state.expo.expo_t = 1000;
  states->push_back(state);
  // state 2
  state.seq_id = 2;
  state.expo.gain = 0.0;
  state.expo.expo_t = 4000;
  states->push_back(state);
  // state 3
  state.seq_id = 3;
  state.expo.gain = 0.0;
  state.expo.expo_t = 16000;
  states->push_back(state);
}

}  // namespace hdr_attr_ctrl
