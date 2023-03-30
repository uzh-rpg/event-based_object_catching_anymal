#include <iostream>
#include <omp.h>
#include <vector>

#include "rpg_dynamic_obstacle_detection/clustering.hpp"

namespace rpg_dynamic_obstacle_detection {
Clustering::Clustering() {}
Clustering::Clustering(float eps, std::vector<float> weights,
                       bool use_rotated_rect)
    : m_eps(eps), weights_(weights), use_rotated_rect_(use_rotated_rect) {
  reset();

  labels_.reserve(number_of_preallocated_elements_);
  ne_.reserve(number_of_preallocated_elements_);
  ne1_.reserve(number_of_preallocated_elements_);
  visited_.reserve(number_of_preallocated_elements_);

  connected_components_rects_.reserve(number_of_preallocated_elements_);
  cluster_velocity_.reserve(number_of_preallocated_elements_);
  cluster_mean_timestamp_.reserve(number_of_preallocated_elements_);
  connected_components_clusters_ = std::vector<std::vector<cv::Point>>(
      number_of_preallocated_elements_,
      std::vector<cv::Point>(number_of_max_expected_points_));
  nonzero_connected_components_.reserve(number_of_max_expected_points_);
  cluster_rects_.reserve(number_of_preallocated_elements_);
}

void Clustering::init(float eps, std::vector<float> weights,
                      bool use_rotated_rect) {
  m_eps = eps;
  weights_ = weights;
  use_rotated_rect_ = use_rotated_rect;

  if (!initialized_) {
    labels_.reserve(number_of_preallocated_elements_);
    ne_.reserve(number_of_preallocated_elements_);
    ne1_.reserve(number_of_preallocated_elements_);
    visited_.reserve(number_of_preallocated_elements_);

    connected_components_rects_.reserve(number_of_preallocated_elements_);
    cluster_velocity_.reserve(number_of_preallocated_elements_);
    cluster_mean_timestamp_.reserve(number_of_preallocated_elements_);
    connected_components_clusters_ = std::vector<std::vector<cv::Point>>(
        number_of_preallocated_elements_,
        std::vector<cv::Point>(number_of_max_expected_points_));
    nonzero_connected_components_.reserve(number_of_max_expected_points_);
    cluster_rects_.reserve(number_of_preallocated_elements_);
    initialized_ = true;
  }
}

void Clustering::initOpticalFlow(size_t window_width, double inc_std,
                                 size_t median_filter_width,
                                 size_t mean_input_filter_width,
                                 size_t mean_output_filter_width) {
  optical_flow_.initialize(window_width, inc_std, median_filter_width,
                           mean_input_filter_width, mean_output_filter_width);
}

void Clustering::cluster(const cv::Mat &img, const cv::Mat &thresh_image) {
  reset();
  connectedComponentsClustering(img, thresh_image);
  //std::cout << "Max label is " << max_label_ << std::endl;
  if (max_label_ > 1) {
    opticalFlow(img, thresh_image);
    prepareClusterData(img);
    wfit();
    label_set_ = minSet(labels_);
    if (std::find(label_set_.begin(), label_set_.end(), -1) !=
        label_set_.end()) {
      label_set_.erase(std::find(label_set_.begin(), label_set_.end(), -1));
    }

    std::vector<size_t> label_idx;
    label_idx.reserve(labels_.size());
    for (int i = 0; i < label_set_.size(); ++i) {
      label_idx.clear();
      if (std::count(labels_.begin(), labels_.end(), i) > 1) {
        for (int32_t k = 0; k < labels_.size(); ++k) {
          if (labels_[k] == i) {
            label_idx.push_back(k);
          }
        }
        for (int k = 1; k < label_idx.size(); ++k) {
          connected_components_clusters_[label_idx[0]].insert(
              connected_components_clusters_[label_idx[0]].end(),
              connected_components_clusters_[label_idx[k]].begin(),
              connected_components_clusters_[label_idx[k]].end());
          connected_components_clusters_[label_idx[k]].clear();
        }
      }
    }
  }
  cluster_rects_.clear();
  for (const auto &cluster : connected_components_clusters_) {
    if (cluster.size() > 0) {
      if (use_rotated_rect_) {
        cluster_rects_.push_back(cv::minAreaRect(cluster));
      } else {
        cv::Rect rect = cv::boundingRect(cluster);
        cv::Point2f center(rect.x + rect.width / 2.0,
                           rect.y + rect.height / 2.0);
        cluster_rects_.push_back(cv::RotatedRect(center, rect.size(), 0.0));
      }
    }
  }
}

Clustering::~Clustering() {}

void Clustering::reset() { labels_.clear(); }

void Clustering::prepare_labels(size_t s) {
  labels_.resize(s);
  for (auto &l : labels_) {
    l = -1;
  }
}

void Clustering::prepareClusterData(const cv::Mat &img) {
  connected_components_rects_.resize(max_label_);
  for (size_t i = 0; i < max_label_; ++i) {
    connected_components_rects_[i] =
        cv::minAreaRect(connected_components_clusters_[i]);
  }
  cluster_velocity_.resize(max_label_);
  cluster_mean_timestamp_.resize(max_label_);
  for (int i = 0; i < max_label_; ++i) {
    cluster_velocity_[i] = Eigen::Vector2f::Zero();
    cluster_mean_timestamp_[i] = 0.0;
    for (const cv::Point &p : connected_components_clusters_[i]) {
      cluster_velocity_[i].x() += optical_flow_u_.at<float>(p);
      cluster_velocity_[i].y() += optical_flow_v_.at<float>(p);
      cluster_mean_timestamp_[i] += img.at<float>(p);
    }
    cluster_velocity_[i].x() /= connected_components_clusters_[i].size();
    cluster_velocity_[i].y() /= connected_components_clusters_[i].size();
    cluster_mean_timestamp_[i] /= connected_components_clusters_[i].size();
  }
}

void Clustering::opticalFlow(const cv::Mat &img, const cv::Mat &thresh_image) {
  if (optical_flow_u_.empty() || optical_flow_v_.empty()) {
    optical_flow_u_ = cv::Mat::zeros(img.rows, img.cols, CV_32FC1);
    optical_flow_v_ = cv::Mat::zeros(img.rows, img.cols, CV_32FC1);
  }
  optical_flow_.getLucasKanadeOpticalFlowReduced(img, optical_flow_u_,
                                                 optical_flow_v_, thresh_image);

}

void Clustering::connectedComponentsClustering(const cv::Mat &img,
                                               const cv::Mat &thresh_image) {
  cv::connectedComponents(thresh_image, connected_components_labels_16bit_, 8,
                          CV_16U);
  double min_label;
  cv::minMaxLoc(connected_components_labels_16bit_, &min_label, &max_label_);
  if (max_label_ > 255)
    LOG(ERROR) << "Too many connectedComponents labels for CV_8UC1.";
  connected_components_labels_16bit_.convertTo(
      connected_components_labels_8bit_, CV_8U);

  cv::findNonZero(connected_components_labels_8bit_,
                  nonzero_connected_components_);

  for (std::vector<cv::Point> &v : connected_components_clusters_)
    v.clear();
  for (const cv::Point &p : nonzero_connected_components_) {
    size_t label = connected_components_labels_8bit_.at<uint8_t>(p) - 1;
    connected_components_clusters_[label].push_back(p);
  }
}

float Clustering::minDistPointRectSqr(const cv::RotatedRect &rect,
                                      const cv::Point &point) {
  float relx = point.x - rect.center.x;
  float rely = point.y - rect.center.y;
  float theta = rect.angle * M_PI / 180.0;
  float rotx = relx * std::cos(-theta) - rely * std::sin(-theta);
  float roty = relx * std::sin(-theta) + rely * std::cos(-theta);
  float dx = std::max(std::fabs(rotx) - rect.size.width / 2.0, 0.0);
  float dy = std::max(std::fabs(roty) - rect.size.height / 2.0, 0.0);
  return dx * dx + dy * dy;
}

float Clustering::minDistance2RotatedRects(const cv::RotatedRect &rect1,
                                           const cv::RotatedRect &rect2) {
  cv::Point2f rect_points[4];
  rect2.points(rect_points);
  std::vector<float> dist(4);
  for (int i = 0; i < 4; ++i) {
    dist[i] = minDistPointRectSqr(rect1, rect_points[i]);
  }
  return std::sqrt(*std::min_element(dist.begin(), dist.end()));
}

size_t Clustering::tragEq(const size_t &row, const size_t &col,
                          const size_t &N) {
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

float Clustering::calc_dist(const uint32_t &x, const uint32_t &y) {
  if (x == y)
    return 0.0;

  std::vector<float> diff(3);
  diff[0] =
      weights_[0] * minDistance2RotatedRects(connected_components_rects_[x],
                                             connected_components_rects_[y]);
  diff[1] = weights_[1] * (cluster_velocity_[x] - cluster_velocity_[y]).norm();
  diff[2] = weights_[2] *
            std::fabs(cluster_mean_timestamp_[x] - cluster_mean_timestamp_[y]);

  return std::accumulate(diff.begin(), diff.end(), 0.0);
}

void Clustering::find_neighbors(uint32_t pid) {
  ne_.clear();
  for (uint32_t j = 0; j < N_; ++j) {
    if (calc_dist(pid, j) <= m_eps) {
      ne_.push_back(j);
    }
  }
}

void Clustering::find_neighbors_1(uint32_t pid) {
  ne1_.clear();
  for (uint32_t j = 0; j < N_; ++j) {
    if (calc_dist(pid, j) <= m_eps) {
      ne1_.push_back(j);
    }
  }
}

void Clustering::dbscan() {
  visited_.clear();
  visited_.resize(labels_.size());

  uint32_t cluster_id = 0;

  for (uint32_t pid = 0; pid < labels_.size(); ++pid) {
    if (!visited_[pid]) {
      visited_[pid] = 1;

      find_neighbors(pid);

      if (ne_.size() >= 1) {
        labels_[pid] = cluster_id;

        for (uint32_t i = 0; i < ne_.size(); ++i) {
          uint32_t nPid = ne_[i];

          if (!visited_[nPid]) {
            visited_[nPid] = 1;

            find_neighbors_1(nPid);

            if (ne1_.size() >= 1) {
              for (const auto &n1 : ne1_) {
                ne_.push_back(n1);
              }
            }
          }

          if (labels_[nPid] == -1) {
            labels_[nPid] = cluster_id;
          }
        }
        ++cluster_id;
      }
    }
  }
}

void Clustering::wfit() {
  N_ = connected_components_rects_.size();
  prepare_labels(N_);
  dbscan();
}
} // namespace rpg_dynamic_obstacle_detection
