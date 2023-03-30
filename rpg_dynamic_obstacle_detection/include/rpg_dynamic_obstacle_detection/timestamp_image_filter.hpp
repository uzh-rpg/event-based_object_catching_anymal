#pragma once

#include <mutex>

#include "opencv2/imgproc/imgproc.hpp"

#include "rpg_dynamic_obstacle_detection/param.hpp"

namespace rpg_dynamic_obstacle_detection {

class TimestampImageFilter {
public:
  TimestampImageFilter(ros::NodeHandle nh, ros::NodeHandle pnh,
                       const cv::Mat &input, std::mutex &timespamp_image_mutex);

  TimestampImageFilter(const cv::Mat &input, std::mutex &timespamp_image_mutex)
      : TimestampImageFilter(ros::NodeHandle(), ros::NodeHandle("~"), input,
                             timespamp_image_mutex) {}
  ~TimestampImageFilter() {}

  cv::Mat &getFilteredImage() { return output_; }

  void filterImage();

private:
  void loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  const cv::Mat &input_;
  cv::Mat output_;
  cv::Mat element_open_;
  cv::Mat element_close_;

  int closing_iterations_;
  int opening_iterations_;

  std::mutex &timespamp_image_mutex_;
};
} // namespace rpg_dynamic_obstacle_detection
