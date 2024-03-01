/*
 * @Author: Shuyang Zhang
 * @Date: 2022-11-18 14:23:39
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2024-03-01 16:22:37
 * @Description:
 *
 * Copyright (c) 2022 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <Spinnaker.h>
#include <ros/ros.h>

#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

namespace hdr_attr_ctrl {

/**
 * enumeration
 */
enum CaptureMode {
  CAP_IDLE = 0,
  CAP_AUTO = 1,
  CAP_SHIM = 2,
  CAP_KIM = 3,
  CAP_HDR = 4
};

/**
 * structure
 */
struct Exposure {
  double expo_t;
  double gain;
};

struct SequenceState {
  uint32_t seq_id;
  Exposure expo;
};

struct FuncSample {
  double expo_val;
  double score;
  double sigma;
};
struct ImageWithInfo {
  double timestamp;
  Exposure expo;
  cv::Mat img;
};

typedef std::vector<SequenceState> SequenceStates;
typedef std::vector<FuncSample> FuncSamples;
typedef std::pair<double, double> Range;
typedef int ExpoState;

/**
 * static function
 */
static double Exposure2ExposureVal(const Exposure &expo) {
  return expo.expo_t * std::pow(10.0, expo.gain / 20.0);
}

static Exposure ExposureVal2Exposure_ExposureFirst(const double &expo_val,
                                                   const double expo_t_max) {
  Exposure expo;
  if (expo_val <= expo_t_max) {
    expo.expo_t = expo_val;
    expo.gain = 0.0;
  } else {
    expo.expo_t = expo_t_max;
    expo.gain = log10(expo_val / expo.expo_t) * 20.0;
  }
  return expo;
}

static CaptureMode String2CaptureMode(const std::string &mode) {
  CaptureMode cap_mode = CaptureMode::CAP_IDLE;
  if (mode.compare("auto") == 0) cap_mode = CaptureMode::CAP_AUTO;
  if (mode.compare("shim") == 0) cap_mode = CaptureMode::CAP_SHIM;
  if (mode.compare("kim") == 0) cap_mode = CaptureMode::CAP_KIM;
  if (mode.compare("hdr") == 0) cap_mode = CaptureMode::CAP_HDR;
  return cap_mode;
}

static std::string CaptureMode2String(const CaptureMode &mode) {
  std::string str;
  switch (mode) {
    case CaptureMode::CAP_AUTO:
      str = "auto";
      break;
    case CaptureMode::CAP_SHIM:
      str = "shim";
      break;
    case CaptureMode::CAP_KIM:
      str = "kim";
      break;
    case CaptureMode::CAP_HDR:
      str = "hdr";
      break;
    case CaptureMode::CAP_IDLE:
    default:
      ROS_ERROR("Unexpected capture mode (%d) is detected...Abort",
                static_cast<int>(mode));
      ros::shutdown();
      break;
  }
  return str;
}

static Exposure CheckExposure(const Exposure &expo, const double &expo_t_max,
                              const double &expo_t_min, const double &gain_max,
                              const double &gain_min) {
  Exposure expo_out = expo;
  if (expo_out.expo_t < expo_t_min) expo_out.expo_t = expo_t_min;
  if (expo_out.expo_t > expo_t_max) expo_out.expo_t = expo_t_max;
  if (expo_out.gain < gain_min) expo_out.gain = gain_min;
  if (expo_out.gain > gain_max) expo_out.gain = gain_max;
  return expo_out;
}

static bool CheckEquality(const Exposure &expo1, const Exposure &expo2) {
  if (fabs(expo1.expo_t - expo2.expo_t) > 20.0) return false;
  if (fabs(expo1.gain - expo2.gain) > 0.1) return false;
  return true;
}

static bool CheckEquality(const SequenceStates &states1,
                          const SequenceStates &states2) {
  if (states1.size() != states2.size()) return false;
  for (size_t i = 0; i < states1.size(); ++i) {
    if (fabs(states1.at(i).expo.expo_t - states2.at(i).expo.expo_t) > 50.0)
      return false;
    if (fabs(states1.at(i).expo.gain - states2.at(i).expo.gain) > 0.1)
      return false;
  }
  return true;
}

static std::string GetSequencerStatesString(
    const SequenceStates &sequence_states) {
  std::string str_states = "";
  for (size_t i = 0; i < sequence_states.size(); ++i) {
    char char_tmp[20];
    snprintf(char_tmp, sizeof(char_tmp), "<%.1f,%.1f>",
             sequence_states.at(i).expo.expo_t,
             sequence_states.at(i).expo.gain);
    std::string str_tmp = char_tmp;
    str_states = str_states + str_tmp;
  }
  return str_states;
}

static bool ExistInVector(const std::vector<int> &vec, const int &value) {
  if (vec.empty()) {
    return false;
  }
  if (std::find(vec.begin(), vec.end(), value) == vec.end()) {
    return false;
  }
  return true;
}

static bool ExistInVector(const std::vector<int> &vec, const int &value,
                          size_t *idx) {
  *idx = -1;
  if (vec.empty()) {
    return false;
  }
  std::vector<int>::const_iterator iter =
      std::find(vec.begin(), vec.end(), value);
  if (iter == vec.end()) {
    return false;
  }
  *idx = std::distance(vec.begin(), iter);
  return true;
}

static bool ExistInVector(const std::vector<std::string> &vec,
                          const std::string &value) {
  if (vec.empty()) {
    return false;
  }
  if (std::find(vec.begin(), vec.end(), value) == vec.end()) {
    return false;
  }
  return true;
}

static void Convert2Mat(const Spinnaker::ImagePtr &image, cv::Mat *img) {
  if (image->GetPixelFormat() ==
      Spinnaker::PixelFormatEnums::PixelFormat_Mono8) {
    // mono image
    *img = cv::Mat(image->GetHeight() + image->GetYPadding(),
                   image->GetWidth() + image->GetXPadding(), CV_8UC1,
                   image->GetData(), image->GetStride());
  } else {
    // old version before spinnaker 3.x.x
    // need cmake version check and ctrl by marco, but currently no
    // Spinnaker_VERSION found

    // change to BGR8 image
    //   Spinnaker::ImagePtr image_convert =
    //       image->Convert(Spinnaker::PixelFormatEnums::PixelFormat_BGR8);
    //   *img = cv::Mat(image_convert->GetHeight() +
    //   image_convert->GetYPadding(),
    //                  image_convert->GetWidth() +
    //                  image_convert->GetXPadding(), CV_8UC3,
    //                  image_convert->GetData(), image_convert->GetStride());
    //   image_convert->Release();
  }
}

static cv::Mat CalcImageGradientMagnitude(const cv::Mat &img) {
  cv::Mat sobel_x, sobel_y, gm;
  cv::Sobel(img, sobel_x, CV_64F, 1, 0, 3);
  cv::Sobel(img, sobel_y, CV_64F, 0, 1, 3);
  cv::magnitude(sobel_x, sobel_y, gm);
  return gm;
}

static double CalcImageGradientMagnitudeSum(const cv::Mat &img) {
  return cv::sum(CalcImageGradientMagnitude(img))[0];
}

static int sub_to_ind(int *coords, int *cumprod, int num_dims) {
  int index = 0;
  int k;
  assert(coords != NULL);
  assert(cumprod != NULL);
  assert(num_dims > 0);
  for (k = 0; k < num_dims; k++) {
    index += coords[k] * cumprod[k];
  }
  return index;
}

static void ind_to_sub(int p, int num_dims, const int size[], int *cumprod,
                       int *coords) {
  int j;
  assert(num_dims > 0);
  assert(coords != NULL);
  assert(cumprod != NULL);
  for (j = num_dims - 1; j >= 0; j--) {
    coords[j] = p / cumprod[j];
    p = p % cumprod[j];
  }
}

static void getLocalEntropyImage(const cv::Mat &gray, cv::Rect *roi,
                                 cv::Mat *entropy) {
  // 1.define nerghbood model,here it's 9*9
  int neighbood_dim = 2;
  int neighbood_size[] = {3, 3};
  // 2.Pad gray_src
  cv::Mat gray_src_mat(gray);
  cv::Mat pad_mat;
  int left = (neighbood_size[0] - 1) / 2;  // 9 = > 4 , 5 => 2 ,  3 => 1
  int right = left;
  int top = (neighbood_size[1] - 1) / 2;
  int bottom = top;
  copyMakeBorder(gray_src_mat, pad_mat, top, bottom, left, right,
                 cv::BORDER_REPLICATE, 0);
  cv::Mat *pad_src = &pad_mat;
  *roi = cv::Rect(roi->x + top, roi->y + left, roi->width, roi->height);

  // 3.initial neighbood object,reference to Matlab build-in neighbood object
  // system
  //         int element_num = roi_rect.area();
  // here,implement a histogram by ourself ,each bin calcalate gray value
  // frequence
  int hist_count[256] = {0};
  int neighbood_num = 1;
  for (int i = 0; i < neighbood_dim; i++) neighbood_num *= neighbood_size[i];

  // neighbood_corrds_array is a neighbors_num-by-neighbood_dim array containing
  // relative offsets
  int *neighbood_corrds_array =
      (int *)malloc(sizeof(int) * neighbood_num * neighbood_dim);  // NOLINT
  // Contains the cumulative product of the image_size array;used in the
  // sub_to_ind and ind_to_sub calculations.
  int *cumprod = (int *)malloc(neighbood_dim * sizeof(*cumprod));  // NOLINT
  cumprod[0] = 1;
  for (int i = 1; i < neighbood_dim; i++)
    cumprod[i] = cumprod[i - 1] * neighbood_size[i - 1];
  int *image_cumprod = (int *)malloc(2 * sizeof(*image_cumprod));  // NOLINT
  image_cumprod[0] = 1;
  image_cumprod[1] = pad_src->cols;
  // initialize neighbood_corrds_array
  int p;
  int q;
  int *coords;
  for (p = 0; p < neighbood_num; p++) {
    coords = neighbood_corrds_array + p * neighbood_dim;
    ind_to_sub(p, neighbood_dim, neighbood_size, cumprod, coords);
    for (q = 0; q < neighbood_dim; q++)
      coords[q] -= (neighbood_size[q] - 1) / 2;
  }
  // initlalize neighbood_offset in use of neighbood_corrds_array
  int *neighbood_offset = (int *)malloc(sizeof(int) * neighbood_num);  // NOLINT
  int *elem;
  for (int i = 0; i < neighbood_num; i++) {
    elem = neighbood_corrds_array + i * neighbood_dim;
    neighbood_offset[i] = sub_to_ind(elem, image_cumprod, 2);
  }

  // 4.calculate entropy for pixel
  uchar *array = (uchar *)pad_src->data;  // NOLINT
  // here,use entroy_table to avoid frequency log function which cost losts of
  // time
  float entroy_table[10];  // 9 = > 82 , 5 => 26, 3 => 10
  const float log2 = log(2.0f);
  entroy_table[0] = 0.0;
  float frequency = 0;
  for (int i = 1; i < 10; i++) {  // 82, 50, 26, 10
    frequency = (float)i / 9;     // 9, 8, 5, 3 // NOLINT
    entroy_table[i] = frequency * (log(frequency) / log2);
  }

  int neighbood_index;
  //        int max_index=pad_src->cols*pad_src->rows;
  float e;
  int current_index = 0;
  int current_index_in_origin = 0;
  for (int y = roi->y; y < roi->height; y++) {
    current_index = y * pad_src->cols;
    current_index_in_origin =
        (y - 1) * gray.cols;  //     neighbood_size[1]  9->4, 7->3, 5->2 3 -> 1
    // cerr << "yy = " << y  << "gray = " << (y - 4) * gray.cols << endl;
    for (int x = roi->x; x < roi->width;
         x++, current_index++, current_index_in_origin++) {
      for (int j = 0; j < neighbood_num; j++) {
        neighbood_index = current_index + neighbood_offset[j];
        hist_count[array[neighbood_index]]++;
      }
      // get entropy
      e = 0;
      for (int k = 0; k < 256; k++) {
        if (hist_count[k] != 0) {
          //                                        int frequency=hist_count[k];
          e -= entroy_table[hist_count[k]];
          hist_count[k] = 0;
        }
      }
      ((float *)entropy->data)[current_index_in_origin] = e;  // NOLINT
    }
  }

  free(neighbood_offset);
  free(image_cumprod);
  free(cumprod);
  free(neighbood_corrds_array);
}

static void CalcImageLocalEntropy(const cv::Mat &img, cv::Mat *entropy) {
  cv::Rect roi(0, 0, img.cols, img.rows);
  cv::Mat dst = cv::Mat::zeros(img.rows, img.cols, CV_32F);
  getLocalEntropyImage(img, &roi, &dst);
  cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX);
  dst.convertTo(*entropy, CV_32F, 1.0);
}

}  // namespace hdr_attr_ctrl
