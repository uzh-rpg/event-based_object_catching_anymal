#include "rpg_dynamic_obstacle_detection/dbscan_wrapper.hpp"

namespace rpg_dynamic_obstacle_detection {

DbscanWrapper::DbscanWrapper(const ros::NodeHandle &nh,
                             const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  loadParameters();
}

void DbscanWrapper::clusterData(const cv::Mat &image, const cv::Mat &u,
                                const cv::Mat &v) {
  arma::mat data;
  if (!getDataMat(data, image, u, v))
    return;

  mlpack::dbscan::DBSCAN<> dbscan(eps_, min_num_points_);
  arma::Row<size_t> assignments;
  const size_t clusters = dbscan.Cluster(data, assignments);
  std::cout << "number of clusters: " << clusters << std::endl;
}

bool DbscanWrapper::getDataMat(arma::mat &data, const cv::Mat &image,
                               const cv::Mat &u, const cv::Mat &v) {
  std::vector<cv::Point> nonzero;
  cv::findNonZero(image, nonzero);
  if (nonzero.size() < min_num_points_)
    return false;

  data = arma::mat(4, nonzero.size(), arma::fill::none);

  for (size_t i = 0; i < nonzero.size(); ++i) {
    data.at(0, i) = (double)nonzero[i].x;
    data.at(1, i) = (double)nonzero[i].y;
    data.at(2, i) = (double)u.at<float>(nonzero[i].y, nonzero[i].x);
    data.at(3, i) = (double)v.at<float>(nonzero[i].y, nonzero[i].x);
  }

  return true;
}

void DbscanWrapper::loadParameters() {
  eps_ = param<double>(pnh_, "dbscan_eps", 1.0);
  min_num_points_ = (size_t)param<int>(pnh_, "dbscan_min_num_points", 5);
}
} // namespace rpg_dynamic_obstacle_detection
