/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-19 13:13:01
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-08-09 16:10:56
 * @Description:
 *
 * Copyright (c) 2022 by Shuyang Zhang, All Rights Reserved.
 */
#include "include/event_handler/event_handler_auto.h"

namespace hdr_attr_ctrl {

EventHandlerAuto::EventHandlerAuto(const ros::NodeHandle &nh,
                                   const Spinnaker::CameraPtr &cam,
                                   const std::shared_ptr<ContextAuto> &context)
    : EventHandler(nh, cam) {
  context_ = context;
  pub_image_raw_ = nh_.advertise<sensor_msgs::Image>(
      context_->ros_topic_name_image_raw.c_str(), 10);
}

void EventHandlerAuto::OnImageEvent(Spinnaker::ImagePtr image) {
  ts_ = ros::Time::now();
  std_msgs::Header header;
  header.stamp = ts_;
  sensor_msgs::ImagePtr img_msg;
  Exposure expo;
  try {
    // check whether image is complete
    if (image->IsIncomplete()) {
      ROS_WARN("Image incomplete with image status %d !",
               image->GetImageStatus());
      return;
    }
    // get attributes
    Spinnaker::ChunkData chunkdata = image->GetChunkData();
    expo_.expo_t = static_cast<double>(chunkdata.GetExposureTime());
    expo_.gain = static_cast<double>(chunkdata.GetGain());
    expo = expo_;
    expo.expo_t += context_->expo_val_offset;
    char attr_msg_char[200];
    snprintf(attr_msg_char, sizeof(attr_msg_char),
             "seq: %u expo: %.0f gain: %.1f", seq_id_, expo.expo_t,
             expo.gain);
    header.frame_id = attr_msg_char;
    // get image
    cv::Mat img_cv;
    Convert2Mat(image, &img_cv);
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
  // ROS_INFO("processed imgae seq_id: %u", seq_id_);
}

}  // namespace hdr_attr_ctrl
