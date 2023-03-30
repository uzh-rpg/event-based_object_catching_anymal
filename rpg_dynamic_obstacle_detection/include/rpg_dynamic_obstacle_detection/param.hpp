#pragma once

#include <ros/ros.h>

namespace rpg_dynamic_obstacle_detection {
template <typename T>
T param(const ros::NodeHandle &nh, const std::string &name,
        const T &defaultValue) {
  if (nh.hasParam(name)) {
    T v;
    nh.param<T>(name, v, defaultValue);
    ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
    return v;
  }
  ROS_WARN_STREAM("Cannot find value for parameter: "
                  << name << ", assigning default: " << defaultValue);
  return defaultValue;
}
} // namespace rpg_dynamic_obstacle_detection
