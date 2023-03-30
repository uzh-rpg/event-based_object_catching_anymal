#pragma once

//#include <boost/algorithm/minmax.hpp>
//#include <boost/numeric/ublas/io.hpp>
//#include <boost/numeric/ublas/matrix.hpp>
//#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <iostream>
#include <vector>

#include "rpg_dynamic_obstacle_detection/average_timer.hpp"
#include "rpg_dynamic_obstacle_detection/common.hpp"
#include "rpg_dynamic_obstacle_detection/optical_flow.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif /* ifndef M_PI */

namespace rpg_dynamic_obstacle_detection {
class Clustering {
public:
  typedef std::vector<float> FeaturesWeights;
  typedef std::vector<float> DistanceMatrix;
  typedef std::vector<uint32_t> Neighbors;
  typedef std::vector<int32_t> Labels;

  Clustering(float eps, std::vector<float> weights, bool use_rotated_rect);
  Clustering();
  ~Clustering();

  void init(float eps, std::vector<float> weights, bool use_rotated_rect);
  void initOpticalFlow(size_t window_width, double inc_std,
                       size_t median_filter_width,
                       size_t mean_input_filter_width,
                       size_t mean_output_filter_width);

  void cluster(const cv::Mat &img, const cv::Mat &thresh_image);

  const std::vector<std::vector<cv::Point>> &getClusterData() const {
    return connected_components_clusters_;
  }

  const std::vector<cv::RotatedRect> &getClusterRects() const {
    return cluster_rects_;
  }

  const cv::Mat &getOpticalFlowU() const { return optical_flow_u_; }
  const cv::Mat &getOpticalFlowV() const { return optical_flow_v_; }

private:
  inline void reset();
  inline void prepare_labels(size_t s);
  inline float calc_dist(const uint32_t &x, const uint32_t &y);
  inline void find_neighbors(uint32_t pid);
  inline void find_neighbors_1(uint32_t pid);
  void dbscan();
  void wfit();
  inline float minDistance2RotatedRects(const cv::RotatedRect &rect1,
                                        const cv::RotatedRect &rect2);
  inline float minDistPointRectSqr(const cv::RotatedRect &rect,
                                   const cv::Point &point);
  void opticalFlow(const cv::Mat &img, const cv::Mat &thresh_image);
  void connectedComponentsClustering(const cv::Mat &img,
                                     const cv::Mat &thresh_image);
  void prepareClusterData(const cv::Mat &img);

  inline size_t tragEq(const size_t &row, const size_t &col, const size_t &N);

  bool use_rotated_rect_;
  bool initialized_ = false;
  float m_eps;

  const size_t number_of_preallocated_elements_ = 255;
  const size_t number_of_max_expected_points_ = 5000;

  Labels labels_;
  Labels label_set_;

  FeaturesWeights weights_;
  // DistanceMatrix distance_matrix_;
  Neighbors ne_;
  Neighbors ne1_;
  std::vector<bool> visited_;
  size_t N_;

  OpticalFlow optical_flow_;
  cv::Mat optical_flow_u_;
  cv::Mat optical_flow_v_;

  cv::Mat connected_components_labels_16bit_;
  cv::Mat connected_components_labels_8bit_;
  std::vector<cv::Point> nonzero_connected_components_;
  std::vector<std::vector<cv::Point>> connected_components_clusters_;
  std::vector<cv::RotatedRect> connected_components_rects_;
  std::vector<cv::RotatedRect> cluster_rects_;
  std::vector<Eigen::Vector2f> cluster_velocity_;
  std::vector<float> cluster_mean_timestamp_;
  double max_label_;
};

std::ostream &operator<<(std::ostream &o, Clustering &d);
} // namespace rpg_dynamic_obstacle_detection
