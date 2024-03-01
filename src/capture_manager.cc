/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-18 14:23:39
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2024-03-01 16:24:00
 * @Description:
 *
 * Copyright (c) 2022 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/capture_manager.h"

namespace hdr_attr_ctrl {
CaptureManager::CaptureManager() {}

CaptureManager::~CaptureManager() {
  // free cameras
  for (size_t i = 0; i < camera_list_.size(); ++i) {
    camera_list_.at(i)->Release();
  }
  // free camlist
  cam_list_.Clear();
  // free system
  system_->ReleaseInstance();
}

void CaptureManager::Init(const ros::NodeHandle &nh) {
  // init ros
  nh_ = nh;
  // init system
  system_ = Spinnaker::System::GetInstance();
  // init cameras
  ROS_INFO("retreiving list of cameras...");
  cam_list_ = system_->GetCameras();
  int num_cam = cam_list_.GetSize();
  ROS_INFO("get %d cameras...", num_cam);
  // get camera serial from GenAPI
  std::vector<int> camera_serial;
  for (int i = 0; i < num_cam; ++i) {
    std::string str_serial =
        GetTLNodeStringValue(cam_list_.GetByIndex(i), "DeviceSerialNumber");
    camera_serial.push_back(std::stoi(str_serial));
  }
  // register cameras
  for (size_t i = 0; i < serial_list_.size(); ++i) {
    size_t idx;
    if (ExistInVector(camera_serial, serial_list_.at(i), &idx)) {
      ModeInit(i, idx);
    } else {
      ROS_FATAL("cam %d not found...", serial_list_.at(i));
      ros::shutdown();
    }
  }
}

void CaptureManager::ModeInit(const size_t &global_id, const size_t &local_id) {
  switch (g_context_list.at(global_id)->cam_cap_mode) {
    case CaptureMode::CAP_AUTO:
      camera_list_.push_back(std::make_shared<CameraAuto>(
          cam_list_.GetByIndex(local_id), nh_, global_id));
      break;
    case CaptureMode::CAP_SHIM:
      camera_list_.push_back(std::make_shared<CameraShim>(
          cam_list_.GetByIndex(local_id), nh_, global_id));
      break;
    case CaptureMode::CAP_KIM:
      camera_list_.push_back(std::make_shared<CameraKim>(
          cam_list_.GetByIndex(local_id), nh_, global_id));
      break;
    case CaptureMode::CAP_HDR:
      camera_list_.push_back(std::make_shared<CameraHDR>(
          cam_list_.GetByIndex(local_id), nh_, global_id));
      break;
    case CaptureMode::CAP_IDLE:
    default:
      ROS_ERROR("    camera %d mode is wrong: <%s>",
                g_context_list.at(global_id)->cam_serial,
                CaptureMode2String(g_context_list.at(global_id)->cam_cap_mode)
                    .c_str());
      ros::shutdown();
      break;
  }
  ROS_INFO(
      "    camera %d is registered, alias: <%s>, sub_ns: <%s>, mode: "
      "<%s>",
      g_context_list.at(global_id)->cam_serial,
      g_context_list.at(global_id)->cam_alias.c_str(),
      g_context_list.at(global_id)->cam_sub_ns.c_str(),
      CaptureMode2String(g_context_list.at(global_id)->cam_cap_mode).c_str());
}

void CaptureManager::Run() {
  // create thread
  for (size_t i = 0; i < camera_list_.size(); ++i) {
    camera_thread_.push_back(
        std::thread(std::bind(RunCameraAcquisitionThread, camera_list_.at(i))));
  }
  for (size_t i = 0; i < camera_thread_.size(); ++i)
    camera_thread_.at(i).join();
}

void CaptureManager::RunCameraAcquisitionThread(
    const std::shared_ptr<Camera> &cam) {
  cam->Run();
}

void CaptureManager::InitContextsFromYAML(
    const std::string &str_filepath_yaml) {
  g_context_list.clear();
  serial_list_.clear();

  ROS_INFO("Load CaptureManager Parameters from...%s",
           str_filepath_yaml.c_str());
  YAML::Node config = YAML::LoadFile(str_filepath_yaml);
  for (const auto &cam : config["camera"]) {
    CaptureMode mode = String2CaptureMode(cam["mode"].as<std::string>());
    int serial = cam["serial"].as<int>();
    serial_list_.push_back(serial);
    // init context by Capture Mode
    switch (mode) {
      case CaptureMode::CAP_AUTO:
        g_context_list.push_back(std::make_shared<ContextAuto>());
        break;
      case CaptureMode::CAP_SHIM:
        g_context_list.push_back(std::make_shared<ContextShim>());
        break;
      case CaptureMode::CAP_KIM:
        g_context_list.push_back(std::make_shared<ContextKim>());
        break;
      case CaptureMode::CAP_HDR:
        g_context_list.push_back(std::make_shared<ContextHDR>());
        break;
      case CaptureMode::CAP_IDLE:
      default:
        ROS_ERROR("Unexpected capture mode (%d) is detected...Abort",
                  static_cast<int>(mode));
        ros::shutdown();
        break;
    }
    g_context_list.back()->Init(cam);
  }
}
};  // namespace hdr_attr_ctrl
