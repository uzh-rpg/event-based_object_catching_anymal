#pragma once

#include <algorithm>
#include <cassert>
#include <deque>
#include <mutex>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/ros.h>

#include <glog/logging.h>

#include "rpg_dynamic_obstacle_detection/param.hpp"

namespace rpg_dynamic_obstacle_detection {

typedef std::deque<dvs_msgs::Event> EventQueue;
typedef std::vector<cv::Point2f> PointVector;

class NormalizedMeanTimestampImage {
public:
  NormalizedMeanTimestampImage(
      const ros::NodeHandle &nh, const ros::NodeHandle &pnh,
      const EventQueue &events, const PointVector &rectified_points,
      const cv::Mat &warp, const size_t &frame_begin, const size_t &frame_end,
      const cv::Matx33f &K_refx, const cv::Size &sensor_size);

  NormalizedMeanTimestampImage(const EventQueue &events,
                               const PointVector &rectified_points,
                               const cv::Mat &warp, const size_t &frame_begin,
                               const size_t &frame_end,
                               const cv::Matx33f &K_refx,
                               const cv::Size &sensor_size)
      : NormalizedMeanTimestampImage(
            ros::NodeHandle(), ros::NodeHandle("~"), events, rectified_points,
            warp, frame_begin, frame_end, K_refx, sensor_size) {}

  ~NormalizedMeanTimestampImage() {}

    ros::Time getMostRecentTime() {
      return most_recent_event_ts_;
  }


    const cv::Mat &getNormalizedMeanTimestampImage() const {
    return normalized_mean_timestamp_image_;
  }
  const cv::Mat &getThresholdedMeanTimestampImage() const {
    return thresholded_mean_timestamp_image_;
  }
  const cv::Mat &getOverThresholdImage() const { return over_threshold_image_; }

  void drawNormalizedMeanTimestampImage();

  void thresholdMeanTimestampImage();

  void removeBelowThreshold();

  void filterImage();
  void meanFilterImage();

  void lock() { timespamp_image_mutex_.lock(); }
  void unlock() { timespamp_image_mutex_.unlock(); }

  void changeThreshold(const float &a, const float &b);
  void changeFilterRaw(bool enable);
  void changeFilterOpen(int iterations, int size);
  void changeFilterClose(int iterations, int size);
  void changeFilterDilate(int iterations, int size);

private:
  void loadParameters();

  inline int int_floor(const float &x) {
    int i = (int)x;     /* truncate */
    return i - (i > x); /* convert trunc to floor */
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  const EventQueue &events_;
  const PointVector &rectified_points_;
  const cv::Mat &warp_;

  const cv::Size &sensor_size_;
  cv::Rect rect_;

  const cv::Matx33f &K_refx_;

  cv::Mat normalized_mean_timestamp_image_;
  cv::Mat thresholded_mean_timestamp_image_;
  cv::Mat over_threshold_image_;

  const size_t &frame_begin_;
  const size_t &frame_end_;

  float thresh_a_;
  float thresh_b_;

  bool raw_image_filter_;
  cv::Mat raw_image_filter_kernel_r_;
  cv::Mat raw_image_filter_kernel_c_;

  cv::Mat element_open_;
  cv::Mat element_close_;
  int closing_iterations_;
  int opening_iterations_;
  int dilate_iterations_;
  cv::Mat dilate_kernel_;
  ros::Time most_recent_event_ts_;
  std::mutex timespamp_image_mutex_;
};
} // namespace rpg_dynamic_obstacle_detection
