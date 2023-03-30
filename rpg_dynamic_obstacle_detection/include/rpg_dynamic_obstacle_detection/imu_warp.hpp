#pragma once

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <glog/logging.h>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <ze/common/ringbuffer.hpp>
#include <ze/common/types.hpp>

#include "rpg_dynamic_obstacle_detection/param.hpp"

//#include <ros_dvs_msgs/EventImuArray.h>

namespace rpg_dynamic_obstacle_detection {

typedef std::deque<dvs_msgs::Event> EventQueue;

class ImuWarp {
public:
  ImuWarp(ros::NodeHandle nh, ros::NodeHandle pnh, const EventQueue &events,
          const size_t &frame_begin, const size_t &frame_end);

  ImuWarp(const EventQueue &events, const size_t &frame_begin,
          const size_t &frame_end)
      : ImuWarp(ros::NodeHandle(), ros::NodeHandle("~"), events, frame_begin,
                frame_end) {}

  ~ImuWarp() {}

  const cv::Mat &getWarp() const { return warp_; };

  bool computeMeanAngularVelocity();

private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  //void imuCallbackMod(const ros_dvs_msgs::EventImuArray::ConstPtr &msg);
  void loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber event_imu_sub_;

  cv::Mat warp_;
  ze::Ringbuffer<ze::real_t, 6, 1000> imu_buffer_;

  const size_t &frame_begin_;
  const size_t &frame_end_;
  const EventQueue &events_;

  std::mutex imu_mutex_;
};
} // namespace rpg_dynamic_obstacle_detection
