#include <boost/algorithm/minmax.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <iostream>
#include <omp.h>
#include <vector>

#include "clustering/dbscan_opt.h"

namespace clustering_opt {
DBSCAN::DBSCAN() {}

static int num_threads_or_default(int nt) {
  if (!nt) {
    return omp_get_max_threads();
  }
  return nt;
}

void DBSCAN::init(float eps, size_t min_elems, int num_threads) {
  timer_fit_ = rpg_dynamic_obstacle_detection::AverageTimer("fit", 10000);
  timer_dist_ = rpg_dynamic_obstacle_detection::AverageTimer("dist", 10000);
  m_eps = eps;
  m_min_elems = min_elems;
  m_num_threads = num_threads_or_default(num_threads);

  m_labels.reserve(number_of_preallocated_elements_);
  distance_matrix_.resize((number_of_preallocated_elements_ *
                           (number_of_preallocated_elements_ + 1)) /
                          2);
  ne_.reserve(number_of_preallocated_elements_);
  ne1_.reserve(number_of_preallocated_elements_);
  visited_.reserve(number_of_preallocated_elements_);
}

DBSCAN::DBSCAN(float eps, size_t min_elems, int num_threads)
    : m_eps(eps), m_min_elems(min_elems),
      m_num_threads(num_threads_or_default(num_threads)) {
  reset();
}

DBSCAN::~DBSCAN() {}

void DBSCAN::reset() { m_labels.clear(); }

void DBSCAN::prepare_labels(size_t s) {
  m_labels.resize(s);
  for (auto &l : m_labels) {
    l = -1;
  }
}

size_t DBSCAN::tragEq(const size_t &row, const size_t &col, const size_t &N) {
  double x, y;
  if (row <= col) {
    y = row;
    x = col;
  } else {
    y = col;
    x = row;
  }
  return (y * (2 * N - y + 1)) / 2 + (x - y - 1);
}

float DBSCAN::calc_dist(const ClusterData &C, const uint32_t &x,
                        const uint32_t &y) {
  float dist;
  std::vector<float> delta;
  delta.reserve(4);
  for (int i = 0; i < 4; ++i) {
    delta.push_back(C(x, i) - C(y, i));
  }
  dist = weights_[0] * std::sqrt(delta[0] * delta[0] + delta[1] * delta[1]) +
         weights_[1] * std::sqrt(delta[2] * delta[2] + delta[3] * delta[3]);

  return dist;
}

void DBSCAN::calc_dist_matrix(const DBSCAN::ClusterData &C) {
  if (C.size1() > number_of_preallocated_elements_) {
    LOG(ERROR) << "Number of preallocated elements to small!";
  }

  DBSCAN::ClusterData cl_d = C;

  omp_set_dynamic(0);
  omp_set_num_threads(m_num_threads);
#pragma omp parallel for
  for (size_t i = 0; i < cl_d.size1(); ++i) {
    for (size_t j = i; j < cl_d.size1(); ++j) {
      if (i != j) {
        ublas::matrix_row<DBSCAN::ClusterData> U(cl_d, i);
        ublas::matrix_row<DBSCAN::ClusterData> V(cl_d, j);

        // Custom distance function.
        std::vector<float> delta;
        delta.reserve(5);
        for (const auto e : (U - V)) {
          delta.push_back(e);
        }
        distance_matrix_[tragEq(i, j, N_)] =
            weights_[0] * std::sqrt(delta[0] * delta[0] + delta[1] * delta[1]) +
            weights_[1] * std::sqrt(delta[2] * delta[2] + delta[3] * delta[3]);
        // weights_[2] * fabs(delta[4]);

      } else {
        distance_matrix_[tragEq(i, j, N_)] = 0.0;
      }
    }
  }
}

void DBSCAN::find_neighbors_opt(const ClusterData &C, uint32_t pid) {
  for (uint32_t j = pid; j < N_; ++j) {
    if (calc_dist(C, pid, j) <= m_eps) {
      ne_.push_back(j);
    }
  }
}

void DBSCAN::find_neighbors_1_opt(const ClusterData &C, uint32_t pid) {
  for (uint32_t j = pid; j < N_; ++j) {
    if (calc_dist(C, pid, j) <= m_eps) {
      ne1_.push_back(j);
    }
  }
}

void DBSCAN::find_neighbors(const DBSCAN::DistanceMatrix &D, uint32_t pid) {
  for (uint32_t j = pid; j < N_; ++j) {
    if (D[tragEq(pid, j, N_)] <= m_eps) {
      ne_.push_back(j);
    }
  }
}

void DBSCAN::find_neighbors_1(const DBSCAN::DistanceMatrix &D, uint32_t pid) {
  for (uint32_t j = pid; j < N_; ++j) {
    if (D[tragEq(pid, j, N_)] <= m_eps) {
      ne1_.push_back(j);
    }
  }
}

void DBSCAN::dbscan_opt(const ClusterData &C) {
  visited_.clear();
  visited_.resize(m_labels.size());

  uint32_t cluster_id = 0;

  for (uint32_t pid = 0; pid < m_labels.size(); ++pid) {
    if (!visited_[pid]) {
      visited_[pid] = 1;

      ne_.clear();
      find_neighbors_opt(C, pid);

      if (ne_.size() >= m_min_elems) {
        m_labels[pid] = cluster_id;

        for (uint32_t i = 0; i < ne_.size(); ++i) {
          uint32_t nPid = ne_[i];

          if (!visited_[nPid]) {
            visited_[nPid] = 1;

            ne1_.clear();
            find_neighbors_1_opt(C, nPid);

            if (ne1_.size() >= m_min_elems) {
              for (const auto &n1 : ne1_) {
                ne_.push_back(n1);
              }
            }
          }

          if (m_labels[nPid] == -1) {
            m_labels[nPid] = cluster_id;
          }
        }
        ++cluster_id;
      }
    }
  }
}

void DBSCAN::dbscan(const DBSCAN::DistanceMatrix &dm) {
  visited_.clear();
  visited_.resize(m_labels.size());

  uint32_t cluster_id = 0;

  for (uint32_t pid = 0; pid < m_labels.size(); ++pid) {
    if (!visited_[pid]) {
      visited_[pid] = 1;

      ne_.clear();
      find_neighbors(dm, pid);

      if (ne_.size() >= m_min_elems) {
        m_labels[pid] = cluster_id;

        for (uint32_t i = 0; i < ne_.size(); ++i) {
          uint32_t nPid = ne_[i];

          if (!visited_[nPid]) {
            visited_[nPid] = 1;

            ne1_.clear();
            find_neighbors_1(dm, nPid);

            if (ne1_.size() >= m_min_elems) {
              for (const auto &n1 : ne1_) {
                ne_.push_back(n1);
              }
            }
          }

          if (m_labels[nPid] == -1) {
            m_labels[nPid] = cluster_id;
          }
        }
        ++cluster_id;
      }
    }
  }
}

void DBSCAN::wfit(const DBSCAN::ClusterData &C,
                  const DBSCAN::FeaturesWeights &W) {
  weights_ = W;
  N_ = C.size1();
  prepare_labels(C.size1());
  /*
  timer_dist_.restart();
  calc_dist_matrix(C);
  timer_dist_.stop();
  timer_fit_.restart();
  dbscan(distance_matrix_);
  timer_fit_.stop();
  LOG_EVERY_N(INFO, 20) << timer_dist_.get_info_string();
  LOG_EVERY_N(INFO, 20) << timer_fit_.get_info_string();
  */
  timer_dist_.restart();
  dbscan_opt(C);
  timer_dist_.stop();
  LOG_EVERY_N(INFO, 20) << timer_dist_.get_info_string();
}

const DBSCAN::Labels &DBSCAN::get_labels() const { return m_labels; }

std::ostream &operator<<(std::ostream &o, DBSCAN &d) {
  o << "[ ";
  for (const auto &l : d.get_labels()) {
    o << " " << l;
  }
  o << " ] " << std::endl;

  return o;
}
} // namespace clustering_opt
