/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-17 23:41:31
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-29 00:09:46
 * @Description:
 *
 * Copyright (c) 2022 by Shuyang Zhang, All Rights Reserved.
 */

#include <ros/ros.h>

#include <memory>
#include <thread>  // NOLINT
#include <vector>

#include <opencv2/opencv.hpp>

#include "include/capture_manager.h"
#include "include/common.h"
#include "include/context/contexts.h"

std::vector<std::shared_ptr<hdr_attr_ctrl::Context>> g_context_list;

int main(int argc, char **argv) {
  ros::init(argc, argv, "acquisition");
  std::string str_filepath_yaml = argv[1];
  ros::NodeHandle nh;
  // init capture manager
  hdr_attr_ctrl::CaptureManager capture_manager;
  capture_manager.InitContextsFromYAML(str_filepath_yaml);
  for (size_t i = 0; i < g_context_list.size(); ++i)
    g_context_list.at(i)->Show();
  capture_manager.Init(nh);
  capture_manager.Run();
  return 0;
}
