#pragma once

#include <boost/algorithm/minmax.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <glog/logging.h>
#include <iostream>
#include <omp.h>
#include <vector>

#include "rpg_dynamic_obstacle_detection/average_timer.hpp"

using namespace boost::numeric;

namespace clustering_opt {
class DBSCAN {
public:
  typedef ublas::vector<float> FeaturesWeights;
  typedef ublas::matrix<float> ClusterData;
  typedef std::vector<float> DistanceMatrix;
  typedef std::vector<uint32_t> Neighbors;
  typedef std::vector<int32_t> Labels;

  DBSCAN(float eps, size_t min_elems, int num_threads = 0);
  DBSCAN();
  ~DBSCAN();

  void init(float eps, size_t min_elems, int num_threads = 0);
  void wfit(const ClusterData &C, const FeaturesWeights &W);
  void reset();

  const Labels &get_labels() const;

private:
  inline void prepare_labels(size_t s);
  // const DistanceMatrix
  inline float calc_dist(const ClusterData &C, const uint32_t &x,
                         const uint32_t &y);
  inline void find_neighbors_opt(const ClusterData &C, uint32_t pid);
  inline void find_neighbors_1_opt(const ClusterData &C, uint32_t pid);
  void dbscan_opt(const ClusterData &C);

  void calc_dist_matrix(const ClusterData &C);
  inline void find_neighbors(const DistanceMatrix &D, uint32_t pid);
  inline void find_neighbors_1(const DistanceMatrix &D, uint32_t pid);
  void dbscan(const DistanceMatrix &dm);

  inline size_t tragEq(const size_t &row, const size_t &col, const size_t &N);

  float m_eps;
  size_t m_min_elems;
  int m_num_threads;

  size_t number_of_preallocated_elements_ = 5000;

  Labels m_labels;

  FeaturesWeights weights_;
  DistanceMatrix distance_matrix_;
  Neighbors ne_;
  Neighbors ne1_;
  std::vector<bool> visited_;
  size_t N_;

  rpg_dynamic_obstacle_detection::AverageTimer timer_dist_;
  rpg_dynamic_obstacle_detection::AverageTimer timer_fit_;
};

std::ostream &operator<<(std::ostream &o, DBSCAN &d);
} // namespace clustering_opt
