#pragma once

#include <sstream>
#include <string>

#include <glog/logging.h>

#include "rpg_dynamic_obstacle_detection/common.hpp"
#include "rpg_dynamic_obstacle_detection/state.hpp"

namespace rpg_dynamic_obstacle_detection {

class MeasurementStatistics {
public:
  MeasurementStatistics(size_t size);
  ~MeasurementStatistics() {}

  void push_back(const std::vector<State> &measurements);
  std::string get_info_string();

private:
  bool stateContainsNAN(const State &s);

  std::vector<double> measurements_vel_x_;
  std::vector<double> measurements_vel_y_;
  std::vector<double> measurements_vel_x_abs_;
  std::vector<double> measurements_vel_y_abs_;
  std::vector<double> measurements_area_;
  std::vector<double> measurements_width_;
  std::vector<double> measurements_height_;
  std::vector<double> measurements_angle_;
  std::vector<double> measurements_timestamp_;
};
} // namespace rpg_dynamic_obstacle_detection
