/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-18 14:23:28
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-08 20:19:35
 * @Description:
 *
 * Copyright (c) 2022 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/camera/camera_kim.h"

namespace hdr_attr_ctrl {
CameraKim::CameraKim(const Spinnaker::CameraPtr &cam_ptr,
                     const ros::NodeHandle &nh, const size_t &global_idx) {
  nh_ = nh;
  cam_ = cam_ptr;
  context_ =
      std::dynamic_pointer_cast<ContextKim>(g_context_list.at(global_idx));
}

void CameraKim::Run() {
  InitCamera(cam_, context_->cam_serial);
  InitCameraForManualExposure();
  InitCameraAttribute();
  RegisterController();
  ros::Duration(0.1).sleep();
  BeginCapture(cam_, context_->cam_serial);
  while (ros::ok()) {
    ros::Duration(0.1).sleep();
  }
}

void CameraKim::RegisterController() {
  try {
    controller_ = std::shared_ptr<EventHandlerKim>(
        new EventHandlerKim(nh_, cam_, context_));
    cam_->RegisterEventHandler(*controller_);
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

void CameraKim::InitCameraForManualExposure() {
  try {
    Spinnaker::GenApi::INodeMap &node_map = cam_->GetNodeMap();
    DisableSequencerMode(node_map);
    ROS_INFO("cam %d: Sequencer mode disable...", context_->cam_serial);
    DisableAutoExposure(node_map);
    ROS_INFO("cam %d: Automatic exposure disabled...", context_->cam_serial);
    DisableAutoGain(node_map);
    ROS_INFO("cam %d: Automatic gain disabled...", context_->cam_serial);
    EnableContinuousAcquisition(node_map);
    ROS_INFO("cam %d: Acquisition mode set to continuous...",
             context_->cam_serial);
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

void CameraKim::InitCameraAttribute() {
  try {
    Spinnaker::GenApi::INodeMap &node_map = cam_->GetNodeMap();
    ChangeExposureTime(node_map, context_->kim_seed_expo_t);
    ChangeGain(node_map, context_->kim_seed_gain);
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

}  // namespace hdr_attr_ctrl
