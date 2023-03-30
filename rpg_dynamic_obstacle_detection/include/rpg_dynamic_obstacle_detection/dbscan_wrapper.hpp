#pragma once

#include "opencv2/core/core.hpp"
#include <mlpack/core.hpp>
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <ros/ros.h>

#include "rpg_dynamic_obstacle_detection/param.hpp"

namespace rpg_dynamic_obstacle_detection {

class DbscanWrapper {
public:
  DbscanWrapper(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  DbscanWrapper() : DbscanWrapper(ros::NodeHandle(), ros::NodeHandle("~")) {}
  ~DbscanWrapper() {}

  void clusterData(const cv::Mat &image, const cv::Mat &u, const cv::Mat &v);

private:
  bool getDataMat(arma::mat &data, const cv::Mat &image, const cv::Mat &u,
                  const cv::Mat &v);
  void loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  arma::mat data_;

  double eps_;
  size_t min_num_points_;
};
} // namespace rpg_dynamic_obstacle_detection
