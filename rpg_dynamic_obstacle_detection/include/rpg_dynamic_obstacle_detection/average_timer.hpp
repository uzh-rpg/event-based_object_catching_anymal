#pragma once

#include <glog/logging.h>
#include <sstream>
#include <fstream>
#include <string>

#include "rpg_dynamic_obstacle_detection/timer.h"
#include "rpg_dynamic_obstacle_detection/common.hpp"

namespace rpg_dynamic_obstacle_detection {
class AverageTimer {
public:
  AverageTimer(std::string name, size_t size) : name_(name) {
    times_.reserve(size);
  }
  
  AverageTimer() {}
  ~AverageTimer() {}

  void restart() {
    timer_.reset();
    timer_.start();
  }

  void stop() {
    timer_.stop();
    times_.push_back(timer_.getMilliseconds());
  }

  double getTime() { return times_.back(); }

  std::vector<double> getTimeVector() { return times_; }

  std::string get_info_string() {
    std::stringstream ss;
    ss << "Timer " << name_ << ":\n\tmean: " << vector_mean(times_)
       << "\n\tstd: " << vector_std(times_)
       << "\n\tmax: " << *std::max_element(times_.begin(), times_.end())
       << "\n\tmin: " << *std::min_element(times_.begin(), times_.end());
    return ss.str();
  }

  void writeTime() {
    std::fstream outfile;
    std::stringstream ss;
    ss << name_ << ".csv";
    outfile.open(ss.str(), std::ios::out | std::ios::app);
    outfile << getTime() << std::endl;
    outfile.close();
  }

private:
  std::string name_;
  Timer timer_;
  std::vector<double> times_;
};
} // namespace rpg_dynamic_obstacle_detection
