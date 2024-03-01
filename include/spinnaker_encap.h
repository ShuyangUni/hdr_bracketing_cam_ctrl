/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-24 19:05:39
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-29 00:07:33
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <ros/ros.h>

#include <string>

#include "Spinnaker.h"
#include "include/common.h"

namespace hdr_attr_ctrl {
static void EnableSequencerMode(const Spinnaker::GenApi::INodeMap &node_map) {
  try {
    Spinnaker::GenApi::CEnumerationPtr ptr_sequencer_mode =
        node_map.GetNode("SequencerMode");
    Spinnaker::GenApi::CEnumEntryPtr ptr_sequencer_mode_on =
        ptr_sequencer_mode->GetEntryByName("On");
    ptr_sequencer_mode->SetIntValue(
        static_cast<int64_t>(ptr_sequencer_mode_on->GetValue()));
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void DisableSequencerMode(const Spinnaker::GenApi::INodeMap &node_map) {
  try {
    // check valid
    Spinnaker::GenApi::CEnumerationPtr ptr_sequencer_configuration_valid =
        node_map.GetNode("SequencerConfigurationValid");
    Spinnaker::GenApi::CEnumEntryPtr ptr_sequencer_configuration_valid_yes =
        ptr_sequencer_configuration_valid->GetEntryByName("Yes");
    // If valid, disable sequencer mode; otherwise, do nothing
    Spinnaker::GenApi::CEnumerationPtr ptr_sequencer_mode =
        node_map.GetNode("SequencerMode");
    if (ptr_sequencer_configuration_valid->GetCurrentEntry()->GetValue() ==
        ptr_sequencer_configuration_valid_yes->GetValue()) {
      Spinnaker::GenApi::CEnumEntryPtr ptr_sequencer_mode_off =
          ptr_sequencer_mode->GetEntryByName("Off");
      ptr_sequencer_mode->SetIntValue(
          static_cast<int64_t>(ptr_sequencer_mode_off->GetValue()));
    }
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void EnableSequencerConfiguration(
    const Spinnaker::GenApi::INodeMap &node_map) {
  try {
    Spinnaker::GenApi::CEnumerationPtr ptr_sequencer_configuration_mode =
        node_map.GetNode("SequencerConfigurationMode");
    Spinnaker::GenApi::CEnumEntryPtr ptr_sequencer_configuration_mode_on =
        ptr_sequencer_configuration_mode->GetEntryByName("On");
    ptr_sequencer_configuration_mode->SetIntValue(
        static_cast<int64_t>(ptr_sequencer_configuration_mode_on->GetValue()));
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void DisableSequencerConfiguration(
    const Spinnaker::GenApi::INodeMap &node_map) {
  try {
    Spinnaker::GenApi::CEnumerationPtr ptr_sequencer_configuration_mode =
        node_map.GetNode("SequencerConfigurationMode");
    Spinnaker::GenApi::CEnumEntryPtr ptr_sequencer_configuration_mode_off =
        ptr_sequencer_configuration_mode->GetEntryByName("Off");
    ptr_sequencer_configuration_mode->SetIntValue(
        static_cast<int64_t>(ptr_sequencer_configuration_mode_off->GetValue()));
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void EnableAutoExposure(const Spinnaker::GenApi::INodeMap &node_map) {
  try {
    Spinnaker::GenApi::CEnumerationPtr ptr_exposure_auto =
        node_map.GetNode("ExposureAuto");
    Spinnaker::GenApi::CEnumEntryPtr ptr_exposure_auto_continuous =
        ptr_exposure_auto->GetEntryByName("Continuous");
    ptr_exposure_auto->SetIntValue(
        static_cast<int64_t>(ptr_exposure_auto_continuous->GetValue()));
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void DisableAutoExposure(const Spinnaker::GenApi::INodeMap &node_map) {
  try {
    Spinnaker::GenApi::CEnumerationPtr ptr_exposure_auto =
        node_map.GetNode("ExposureAuto");
    Spinnaker::GenApi::CEnumEntryPtr ptr_exposure_auto_off =
        ptr_exposure_auto->GetEntryByName("Off");
    ptr_exposure_auto->SetIntValue(
        static_cast<int64_t>(ptr_exposure_auto_off->GetValue()));
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void EnableAutoGain(const Spinnaker::GenApi::INodeMap &node_map) {
  try {
    Spinnaker::GenApi::CEnumerationPtr ptr_gain_auto =
        node_map.GetNode("GainAuto");
    Spinnaker::GenApi::CEnumEntryPtr ptr_gain_auto_continuous =
        ptr_gain_auto->GetEntryByName("Continuous");
    ptr_gain_auto->SetIntValue(
        static_cast<int64_t>(ptr_gain_auto_continuous->GetValue()));
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void DisableAutoGain(const Spinnaker::GenApi::INodeMap &node_map) {
  try {
    Spinnaker::GenApi::CEnumerationPtr ptr_gain_auto =
        node_map.GetNode("GainAuto");
    Spinnaker::GenApi::CEnumEntryPtr ptr_gain_auto_off =
        ptr_gain_auto->GetEntryByName("Off");
    ptr_gain_auto->SetIntValue(
        static_cast<int64_t>(ptr_gain_auto_off->GetValue()));
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void EnableContinuousAcquisition(
    const Spinnaker::GenApi::INodeMap &node_map) {
  try {
    Spinnaker::GenApi::CEnumerationPtr ptr_acquisition_mode =
        node_map.GetNode("AcquisitionMode");
    Spinnaker::GenApi::CEnumEntryPtr ptr_acquisition_mode_continuous =
        ptr_acquisition_mode->GetEntryByName("Continuous");
    int64_t acquisition_mode_continuous =
        ptr_acquisition_mode_continuous->GetValue();
    ptr_acquisition_mode->SetIntValue(acquisition_mode_continuous);
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void SetSequencerState(const Spinnaker::GenApi::INodeMap &node_map,
                              const uint32_t &state_vol,
                              const SequenceState &state) {
  try {
    // select the current sequence number
    Spinnaker::GenApi::CIntegerPtr ptr_sequencer_set_selector =
        node_map.GetNode("SequencerSetSelector");
    ptr_sequencer_set_selector->SetValue(state.seq_id);
    // set exposure time; exposure time recorded in microseconds
    Spinnaker::GenApi::CFloatPtr ptr_exposure_time =
        node_map.GetNode("ExposureTime");
    double exposure_time_max = ptr_exposure_time->GetMax();
    double exposure_time_to_set = state.expo.expo_t;
    if (exposure_time_to_set > exposure_time_max) {
      exposure_time_to_set = exposure_time_max;
    }
    ptr_exposure_time->SetValue(exposure_time_to_set);
    // set gain; gain recorded in decibels
    Spinnaker::GenApi::CFloatPtr ptr_gain = node_map.GetNode("Gain");
    double gain_max = ptr_gain->GetMax();
    double gain_to_set = state.expo.gain;
    if (gain_to_set > gain_max) {
      gain_to_set = gain_max;
    }
    ptr_gain->SetValue(gain_to_set);
    // set the trigger type for the current state
    Spinnaker::GenApi::CEnumerationPtr ptr_sequencer_trigger_source =
        node_map.GetNode("SequencerTriggerSource");
    Spinnaker::GenApi::CEnumEntryPtr ptr_sequencer_trigger_source_framestart =
        ptr_sequencer_trigger_source->GetEntryByName("FrameStart");
    ptr_sequencer_trigger_source->SetIntValue(static_cast<int64_t>(
        ptr_sequencer_trigger_source_framestart->GetValue()));
    // set the next state in the sequence
    Spinnaker::GenApi::CIntegerPtr ptr_sequencer_set_next =
        node_map.GetNode("SequencerSetNext");
    if (state.seq_id + 1 == state_vol) {
      ptr_sequencer_set_next->SetValue(0);
    } else {
      ptr_sequencer_set_next->SetValue(state.seq_id + 1);
    }
    // save current state
    Spinnaker::GenApi::CCommandPtr ptr_sequencer_set_save =
        node_map.GetNode("SequencerSetSave");
    ptr_sequencer_set_save->Execute();
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void ChangeExposureTime(const Spinnaker::GenApi::INodeMap &node_map,
                               const double &expo_t) {
  try {
    Spinnaker::GenApi::CFloatPtr ptr_expo = node_map.GetNode("ExposureTime");
    if (!IsAvailable(ptr_expo) || !IsWritable(ptr_expo))
      ROS_ERROR("Unable to set exposure time. Aborting...");
    ptr_expo->SetValue(expo_t);
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void ChangeGain(const Spinnaker::GenApi::INodeMap &node_map,
                       const double &gain) {
  try {
    Spinnaker::GenApi::CFloatPtr ptr_gain = node_map.GetNode("Gain");
    if (!IsAvailable(ptr_gain) || !IsWritable(ptr_gain))
      ROS_ERROR("Unable to set gain. Aborting...");
    ptr_gain->SetValue(gain);
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void InitCameraForSequencer(
    const Spinnaker::GenApi::INodeMap &node_map) {
  try {
    DisableSequencerMode(node_map);
    ROS_INFO("Sequencer mode disable...");
    DisableAutoExposure(node_map);
    ROS_INFO("Automatic exposure disabled...");
    DisableAutoGain(node_map);
    ROS_INFO("Automatic gain disabled...");
    EnableContinuousAcquisition(node_map);
    ROS_INFO("Acquisition mode set to continuous...");
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void ConfigureSequencer(const Spinnaker::GenApi::INodeMap &node_map,
                               const SequenceStates &states) {
  if (states.size() == 0) {
    ROS_WARN("sequence state list is empty...");
    return;
  }
  try {
    DisableSequencerMode(node_map);
    ROS_INFO("Sequencer mode disable...");
    EnableSequencerConfiguration(node_map);
    ROS_INFO("Sequencer configuration mode enabled...");
    // configure sequencer
    for (size_t i = 0; i < states.size(); ++i)
      SetSequencerState(node_map, states.size(), states.at(i));
    DisableSequencerConfiguration(node_map);
    ROS_INFO("Sequencer configuration mode disabled...");
    EnableSequencerMode(node_map);
    ROS_INFO("Sequencer mode enabled...");
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static std::string GetTLNodeStringValue(const Spinnaker::CameraPtr &cam_ptr,
                                        const std::string &node_string) {
  Spinnaker::GenApi::INodeMap &node_map = cam_ptr->GetTLDeviceNodeMap();
  Spinnaker::GenApi::CStringPtr ptr_node_value =
      node_map.GetNode(node_string.c_str());
  if (!Spinnaker::GenApi::IsAvailable(ptr_node_value) ||
      !Spinnaker::GenApi::IsReadable(ptr_node_value)) {
    ROS_FATAL("node %s not readable", node_string.c_str());
    ros::shutdown();
  }
  return std::string(ptr_node_value->GetValue());
}

static void ReleaseCamera(Spinnaker::CameraPtr cam_ptr) {
  // end acquisition
  cam_ptr->EndAcquisition();
  // deinit
  cam_ptr->DeInit();
  cam_ptr = nullptr;
}

static void InitCamera(const Spinnaker::CameraPtr &cam_ptr, const int &serial) {
  try {
    if (cam_ptr->IsInitialized()) {
      ROS_WARN("cam %d: has already been initialized. deinitializing...",
               serial);
      cam_ptr->EndAcquisition();
      cam_ptr->DeInit();
    }
    cam_ptr->Init();
  } catch (const Spinnaker::Exception &e) {
    std::string error_str = e.what();
    ROS_FATAL("spinnaker exception %s...", error_str.c_str());
    ros::shutdown();
  }
}

static void BeginCapture(const Spinnaker::CameraPtr &cam_ptr,
                         const int &serial) {
  cam_ptr->BeginAcquisition();
  ROS_INFO("cam %d: Start acquisition...", serial);
}

static void EndCapture(const Spinnaker::CameraPtr &cam_ptr, const int &serial) {
  cam_ptr->EndAcquisition();
  ROS_INFO("cam %d: End acquisition...", serial);
}

}  // namespace hdr_attr_ctrl
