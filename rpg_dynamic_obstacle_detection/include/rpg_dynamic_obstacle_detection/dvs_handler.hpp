#pragma once

#include <deque>
#include <mutex>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
//#include <ros_dvs_msgs/EventImuArray.h>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>

#include <glog/logging.h>

#include "rpg_dynamic_obstacle_detection/param.hpp"

namespace rpg_dynamic_obstacle_detection {

typedef std::deque<dvs_msgs::Event> EventQueue;
typedef std::vector<cv::Point2f> PointVector;

class DvsHandler {
public:
  DvsHandler(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  DvsHandler() : DvsHandler(ros::NodeHandle(), ros::NodeHandle("~")) {}
  ~DvsHandler();

  bool needNewFrame();

  const EventQueue &getEventQueue() { return events_; }
  const PointVector &getRectifiedPoints() { return rectified_points_; };
  const cv::Matx33f &getKRefx() { return K_refx_; }
  const cv::Size &getSensorSize() { return sensor_size_; }
  const size_t &getFrameBegin() { return frame_begin_; }
  const size_t &getFrameEnd() { return frame_end_; }

  cv::Mat getCameraMatrix() { return K_ref_; }

private:
  void loadParameters();
  void cameraCalibration();
  bool needNewFrameMilliseconds();
  void eventCallback(const dvs_msgs::EventArray::ConstPtr &event_array);
  //void eventCallbackMod(const ros_dvs_msgs::EventImuArray::ConstPtr &msg);
  void clearEventQueue();
  void init(int, int);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber event_sub_;
  ros::Subscriber event_imu_sub_;

  cv::Size sensor_size_;
  size_t width_, height_;

  float fx_, fy_, cx_, cy_;
  cv::Mat K_ref_;
  cv::Matx33f K_refx_;

  EventQueue events_;

  size_t last_event_previous_frame_;
  ros::Time stamp_last_event_previous_frame_;

  PointVector points_;
  PointVector rectified_points_;

  std::string url_;

  int min_num_events_per_frame_;
  size_t frame_begin_, frame_end_;

  double frame_size_;
  double min_step_size_;

  size_t event_history_size_;

  // std::mutex& event_mutex_;
};
} // namespace rpg_dynamic_obstacle_detection
