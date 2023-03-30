#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <numeric>

namespace rpg_dynamic_obstacle_detection {
template <typename T> inline T square(const T &var) { return var * var; }

inline double boundedAngle(const double &input) {
  // Returns the angle in [-pi,pi].
  double out = std::atan2(std::sin(input), std::cos(input));
  return out;
}

template <typename T>
inline bool containsDuplicates(const std::vector<T> &input) {
  std::vector<T> input_copy = input;
  std::sort(input_copy.begin(), input_copy.end());
  auto adjacent_it = std::adjacent_find(input_copy.begin(), input_copy.end());
  if (adjacent_it == input_copy.end())
    return false;
  else
    return true;
}

template <typename T> inline T vector_sum(const std::vector<T> &input) {
  return std::accumulate(input.begin(), input.end(), (T)0.0);
}

inline double vector_mean(const std::vector<double> &vec) {
  if (vec.size() > 0) {
    return std::accumulate(vec.begin(), vec.end(), 0.0) / (double)vec.size();
  } else {
    return 0.0;
  }
}

inline double vector_std(const std::vector<double> &vec) {
  if (vec.size() > 0) {
    double mean = vector_mean(vec);
    double sq_sum =
        std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / vec.size() - mean * mean);
    return stdev;
  } else {
    return 0.0;
  }
}

inline bool eigenMatConstainsNAN(const Eigen::MatrixXd &mat) {
  for (size_t r = 0; r < mat.rows(); ++r) {
    for (size_t c = 0; c < mat.cols(); ++c) {
      if (std::isnan(mat(r, c)))
        return true;
    }
  }
  return false;
}

template <typename T> inline bool matConstainsNAN(const cv::Mat &mat) {
  for (size_t r = 0; r < mat.rows; ++r) {
    for (size_t c = 0; c < mat.cols; ++c) {
      if (std::isnan(mat.at<T>(r, c)))
        return true;
    }
  }
  return false;
}

template <typename T> inline std::vector<T> minSet(std::vector<T> vec) {
  std::vector<T> min_set;
  for (T v : vec) {
    if (!(std::find(min_set.begin(), min_set.end(), v) != min_set.end())) {
      min_set.push_back(v);
    }
  }
  return min_set;
}

} // namespace rpg_dynamic_obstacle_detection
