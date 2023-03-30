#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>

#include <glog/logging.h>

#include "rpg_dynamic_obstacle_detection/average_timer.hpp"
#include "rpg_dynamic_obstacle_detection/common.hpp"
#include "rpg_dynamic_obstacle_detection/param.hpp"

namespace rpg_dynamic_obstacle_detection {
class OpticalFlow {
public:
  OpticalFlow();
  ~OpticalFlow() {}

  bool getLucasKanadeOpticalFlow(const cv::Mat &frame_new, cv::Mat &u,
                                 cv::Mat &v);
  bool getLucasKanadeOpticalFlow(const cv::Mat &frame_new, cv::Mat &u,
                                 cv::Mat &v, const cv::Mat &thresh_image);
  bool getLucasKanadeOpticalFlowReduced(const cv::Mat &frame_new, cv::Mat &u,
                                        cv::Mat &v,
                                        const cv::Mat &thresh_image);
  void initialize(size_t window_width, double inc_std,
                  size_t median_filter_width, size_t mean_input_filter_width,
                  size_t mean_output_filter_width);

private:
  inline bool containsRect(const cv::Mat &mat, const cv::Rect &rect);
  inline cv::Mat getFx(const cv::Mat &src1, const cv::Mat &src2);
  inline cv::Mat getFy(const cv::Mat &src1, const cv::Mat &src2);
  inline cv::Mat getFt(const cv::Mat &src1, const cv::Mat &src2);
  inline cv::Mat getWindowSumMat(const cv::Mat &m, const size_t &wwh);
  void lucasKanadeOpticalFlow(const cv::Mat &img1, const cv::Mat &img2,
                              cv::Mat &u, cv::Mat &v);
  bool inclusionRect(const cv::Mat &input);
  void meanWindowFilter(cv::Mat &src, const size_t &window_width);

  bool initialized_;

  size_t window_width_;
  uint8_t wwh_;
  cv::Mat frame_prev_;
  cv::Mat frame_new_;

  cv::Mat kernel_fx_;
  cv::Mat kernel_fy_;
  cv::Mat kernel_ft1_;
  cv::Mat kernel_ft2_;

  cv::Mat kernel_fx_r_;
  cv::Mat kernel_fy_r_;
  cv::Mat kernel_ft1_r_;
  cv::Mat kernel_ft2_r_;
  cv::Mat kernel_fx_c_;
  cv::Mat kernel_fy_c_;
  cv::Mat kernel_ft1_c_;
  cv::Mat kernel_ft2_c_;

  cv::Mat kernel_sum_;

  cv::Rect inclusion_rect_;
  cv::Point2f mean_prev_;
  cv::Point2f std_prev_;
  float inc_std_;
  bool last_frame_empty_;

  size_t median_filter_width_;
  size_t mean_input_filter_width_;
  size_t mean_output_filter_width_;

  rpg_dynamic_obstacle_detection::AverageTimer timer_;
};
} // namespace rpg_dynamic_obstacle_detection
