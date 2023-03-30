#include "rpg_dynamic_obstacle_detection/measurement_statistics.hpp"

namespace rpg_dynamic_obstacle_detection {
MeasurementStatistics::MeasurementStatistics(size_t size) {
  measurements_vel_x_.reserve(size);
  measurements_vel_y_.reserve(size);
  measurements_vel_x_abs_.reserve(size);
  measurements_vel_y_abs_.reserve(size);
  measurements_area_.reserve(size);
  measurements_width_.reserve(size);
  measurements_height_.reserve(size);
  measurements_angle_.reserve(size);
  measurements_timestamp_.reserve(size);
}

void MeasurementStatistics::push_back(const std::vector<State> &measurements) {
  for (auto m : measurements) {
    if (stateContainsNAN(m)) {
      LOG(ERROR) << "Measurement contains NAN!";
    }
    measurements_vel_x_.push_back(m.vel(0));
    measurements_vel_y_.push_back(m.vel(1));
    measurements_vel_x_abs_.push_back(std::fabs(m.vel(0)));
    measurements_vel_y_abs_.push_back(std::fabs(m.vel(1)));
    measurements_area_.push_back(m.area);
    measurements_width_.push_back(m.width);
    measurements_height_.push_back(m.height);
    measurements_angle_.push_back(m.angle);
    measurements_timestamp_.push_back(m.mean_timestamp);
  }
}

std::string MeasurementStatistics::get_info_string() {
  std::stringstream ss;
  ss << "Measurments:\nVelocity:\nmean: " << vector_mean(measurements_vel_x_)
     << " | " << vector_mean(measurements_vel_y_)
     << "\nStd: " << vector_std(measurements_vel_x_) << " | "
     << vector_std(measurements_vel_y_)
     << "\nVelocity Abs:\nmean: " << vector_mean(measurements_vel_x_abs_)
     << " | " << vector_mean(measurements_vel_y_abs_)
     << "\nStd: " << vector_std(measurements_vel_x_abs_) << " | "
     << vector_std(measurements_vel_y_abs_)
     << "\nArea:\n\tmean: " << vector_mean(measurements_area_)
     << "\n\tStd: " << vector_std(measurements_area_)
     << "\nWidth:\n\tmean: " << vector_mean(measurements_width_)
     << "\n\tStd: " << vector_std(measurements_width_)
     << "\nHeight:\n\tmean: " << vector_mean(measurements_height_)
     << "\n\tStd: " << vector_std(measurements_height_)
     << "\nAngle:\n\tmean: " << vector_mean(measurements_angle_)
     << "\n\tStd: " << vector_std(measurements_angle_)
     << "\nTimestamp:\n\tmean: " << vector_mean(measurements_timestamp_)
     << "\n\tStd: " << vector_std(measurements_timestamp_) << "\n\tRange: "
     << *std::min_element(measurements_timestamp_.begin(),
                          measurements_timestamp_.end())
     << " | "
     << *std::max_element(measurements_timestamp_.begin(),
                          measurements_timestamp_.end());
  return ss.str();
}

bool MeasurementStatistics::stateContainsNAN(const State &s) {
  if (std::isnan(s.pos(0)))
    return true;
  if (std::isnan(s.pos(1)))
    return true;
  if (std::isnan(s.vel(0)))
    return true;
  if (std::isnan(s.vel(1)))
    return true;
  if (std::isnan(s.area))
    return true;
  if (std::isnan(s.angle))
    return true;
  if (std::isnan(s.mean_timestamp))
    return true;
  return false;
}
} // namespace rpg_dynamic_obstacle_detection
