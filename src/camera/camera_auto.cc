/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-18 14:23:28
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-29 00:17:04
 * @Description:
 *
 * Copyright (c) 2022 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/camera/camera_auto.h"

namespace hdr_attr_ctrl {
CameraAuto::CameraAuto(const Spinnaker::CameraPtr &cam_ptr,
                       const ros::NodeHandle &nh, const size_t &global_idx) {
  nh_ = nh;
  cam_ = cam_ptr;
  context_ =
      std::dynamic_pointer_cast<ContextAuto>(g_context_list.at(global_idx));
}

void CameraAuto::Run() {
  InitCamera(cam_, context_->cam_serial);
  InitCameraForAutoExposure();
  RegisterController();
  ros::Duration(0.01).sleep();
  BeginCapture(cam_, context_->cam_serial);
  while (ros::ok()) {
    ros::Duration(0.1).sleep();
  }
}

void CameraAuto::RegisterController() {
  try {
    controller_ = std::shared_ptr<EventHandlerAuto>(
        new EventHandlerAuto(nh_, cam_, context_));
    cam_->RegisterEventHandler(*controller_);
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

void CameraAuto::InitCameraForAutoExposure() {
  try {
    Spinnaker::GenApi::INodeMap &node_map = cam_->GetNodeMap();
    DisableSequencerMode(node_map);
    ROS_INFO("cam %d: Sequencer mode disable...", context_->cam_serial);
    EnableAutoExposure(node_map);
    ROS_INFO("cam %d: Automatic exposure enabled...", context_->cam_serial);
    EnableAutoGain(node_map);
    ROS_INFO("cam %d: Automatic gain enabled...", context_->cam_serial);
    EnableContinuousAcquisition(node_map);
    ROS_INFO("cam %d: Acquisition mode set to continuous...",
             context_->cam_serial);
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

}  // namespace hdr_attr_ctrl
