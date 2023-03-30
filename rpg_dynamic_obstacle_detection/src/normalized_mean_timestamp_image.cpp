#include "rpg_dynamic_obstacle_detection/normalized_mean_timestamp_image.hpp"

namespace rpg_dynamic_obstacle_detection {

NormalizedMeanTimestampImage::NormalizedMeanTimestampImage(
    const ros::NodeHandle &nh, const ros::NodeHandle &pnh,
    const EventQueue &events, const PointVector &rectified_points,
    const cv::Mat &warp, const size_t &frame_begin, const size_t &frame_end,
    const cv::Matx33f &K_refx, const cv::Size &sensor_size)
    : nh_(nh), pnh_(pnh), events_(events), rectified_points_(rectified_points),
      warp_(warp), frame_begin_(frame_begin), frame_end_(frame_end),
      K_refx_(K_refx), sensor_size_(sensor_size) {
  loadParameters();

  // raw_image_filter_kernel_r_ = cv::Mat::zeros(1, 5, CV_32FC1);
  // raw_image_filter_kernel_r_.at<float>(0, 0) = 0.06136;
  // raw_image_filter_kernel_r_.at<float>(0, 1) = 0.24477;
  // raw_image_filter_kernel_r_.at<float>(0, 2) = 0.38774;
  // raw_image_filter_kernel_r_.at<float>(0, 3) = 0.24477;
  // raw_image_filter_kernel_r_.at<float>(0, 4) = 0.06136;
  // raw_image_filter_kernel_c_ = cv::Mat::zeros(5, 1, CV_32FC1);
  // raw_image_filter_kernel_c_.at<float>(0, 0) = 0.06136;
  // raw_image_filter_kernel_c_.at<float>(1, 0) = 0.24477;
  // raw_image_filter_kernel_c_.at<float>(2, 0) = 0.38774;
  // raw_image_filter_kernel_c_.at<float>(3, 0) = 0.24477;
  // raw_image_filter_kernel_c_.at<float>(4, 0) = 0.06136;
  // raw_image_filter_kernel_r_ = cv::Mat::zeros(1, 3, CV_32FC1);
  // raw_image_filter_kernel_r_.at<float>(0, 0) = 0.25;
  // raw_image_filter_kernel_r_.at<float>(0, 1) = 0.5;
  // raw_image_filter_kernel_r_.at<float>(0, 2) = 0.25;
  // raw_image_filter_kernel_c_ = cv::Mat::zeros(3, 1, CV_32FC1);
  // raw_image_filter_kernel_c_.at<float>(0, 0) = 0.25;
  // raw_image_filter_kernel_c_.at<float>(1, 0) = 0.5;
  // raw_image_filter_kernel_c_.at<float>(2, 0) = 0.25;
  raw_image_filter_kernel_r_ = cv::Mat::zeros(1, 7, CV_32FC1);
  raw_image_filter_kernel_r_.at<float>(0, 0) = 2.0/301.0;
  raw_image_filter_kernel_r_.at<float>(0, 1) = 22.0/301.0;
  raw_image_filter_kernel_r_.at<float>(0, 2) = 97.0/301.0;
  raw_image_filter_kernel_r_.at<float>(0, 3) = 159.0/301.0;
  raw_image_filter_kernel_r_.at<float>(0, 4) = 97.0/301.0;
  raw_image_filter_kernel_r_.at<float>(0, 5) = 22.0/301.0;
  raw_image_filter_kernel_r_.at<float>(0, 6) = 2.0/301.0;
  raw_image_filter_kernel_c_ = cv::Mat::zeros(7, 1, CV_32FC1);
  raw_image_filter_kernel_c_.at<float>(0, 0) = 2.0/301.0;
  raw_image_filter_kernel_c_.at<float>(1, 0) = 22.0/301.0;
  raw_image_filter_kernel_c_.at<float>(2, 0) = 97.0/301.0;
  raw_image_filter_kernel_c_.at<float>(3, 0) = 159.0/301.0;
  raw_image_filter_kernel_c_.at<float>(4, 0) = 97.0/301.0;
  raw_image_filter_kernel_r_.at<float>(5, 0) = 22.0/301.0;
  raw_image_filter_kernel_r_.at<float>(6, 0) = 2.0/301.0;
  

  rect_ = cv::Rect(0, 0, sensor_size_.width - 1, sensor_size_.height - 1);
  normalized_mean_timestamp_image_ = cv::Mat::zeros(sensor_size_, CV_32FC1);
  thresholded_mean_timestamp_image_ = cv::Mat::zeros(sensor_size_, CV_8UC1);
  over_threshold_image_ = cv::Mat::zeros(sensor_size_, CV_32FC1);
}

void NormalizedMeanTimestampImage::changeThreshold(const float &a,
                                                   const float &b) {
  thresh_a_ = a;
  thresh_b_ = b;
}
void NormalizedMeanTimestampImage::changeFilterRaw(bool enable) {
  raw_image_filter_ = enable;
}
void NormalizedMeanTimestampImage::changeFilterOpen(int iterations, int size) {
  if (size % 2 == 0)
    --size;
  element_open_ =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
  opening_iterations_ = iterations;
}
void NormalizedMeanTimestampImage::changeFilterClose(int iterations, int size) {
  if (size % 2 == 0)
    --size;
  element_close_ =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
  closing_iterations_ = iterations;
}
void NormalizedMeanTimestampImage::changeFilterDilate(int iterations,
                                                      int size) {
  if (size % 2 == 0)
    --size;
  dilate_kernel_ =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
  dilate_iterations_ = iterations;
}

void NormalizedMeanTimestampImage::removeBelowThreshold() {
  std::vector<cv::Point> nonzeros;
  cv::findNonZero(thresholded_mean_timestamp_image_, nonzeros);
  over_threshold_image_ = cv::Mat::zeros(sensor_size_, CV_32FC1);
  for (auto p : nonzeros) {
    over_threshold_image_.at<float>(p.y, p.x) =
        normalized_mean_timestamp_image_.at<float>(p.y, p.x);
  }
}

void NormalizedMeanTimestampImage::thresholdMeanTimestampImage() {
  float warp_norm = std::sqrt(warp_.at<double>(0) * warp_.at<double>(0) +
                              warp_.at<double>(1) * warp_.at<double>(1) +
                              warp_.at<double>(2) * warp_.at<double>(2));
  float threshold = thresh_a_ * warp_norm + thresh_b_;
  cv::threshold(normalized_mean_timestamp_image_,
                thresholded_mean_timestamp_image_, threshold, 255,
                cv::THRESH_BINARY);
  thresholded_mean_timestamp_image_.convertTo(thresholded_mean_timestamp_image_,
                                              CV_8UC1, 255);
}

void NormalizedMeanTimestampImage::drawNormalizedMeanTimestampImage() {
  if (frame_end_ < frame_begin_) {
    LOG(WARNING) << "frame_begin_ larger than frame_end_";
    return;
  }

  if (std::distance(events_.begin(), events_.end()) < frame_end_) {
    LOG(WARNING) << "frame_end exceeds event queue";
    return;
  }
  const EventQueue::const_iterator ev_first = events_.begin() + frame_begin_;
  const EventQueue::const_iterator ev_last = events_.begin() + frame_end_;

  if (rect_.height <= 0 || rect_.width <= 0)
    rect_ = cv::Rect(0, 0, sensor_size_.width - 1, sensor_size_.height - 1);

  if (normalized_mean_timestamp_image_.cols == 0 ||
      normalized_mean_timestamp_image_.rows == 0)
    normalized_mean_timestamp_image_ = cv::Mat::zeros(sensor_size_, CV_32FC1);

  CHECK((warp_.cols == 3 && warp_.rows == 1) ||
        (warp_.cols == 1 && warp_.rows == 3));

  normalized_mean_timestamp_image_ = cv::Scalar(0);

  cv::Mat event_count = cv::Mat::zeros(sensor_size_, CV_32FC1);
  
  cv::Matx31f w(warp_.at<double>(0), warp_.at<double>(1), warp_.at<double>(2));
  // TODO erase THIS - trying out zero warps
  // cv::Matx31f w(0.0, 0.0, 0.0);

  cv::Matx33f I3 = cv::Matx33f::eye();
  cv::Matx33f Sw(0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0);

  cv::Matx33f H_ref_begin;
  const double delta_t = (ev_last->ts - ev_first->ts).toSec();
  cv::Mat R;
  cv::Rodrigues(warp_ * delta_t, R);
  cv::Matx33f R_end_begin(R);
  H_ref_begin = R_end_begin;

  // Split the data into batches of K events (with K being a small number)
  // and use the same timestamp (the first event timestamp) for the entire batch
  const double t0 = ev_first->ts.toSec();

  EventQueue::const_iterator e = ev_first;

  static constexpr size_t batch_size = 64;
  size_t index_batch = 0;
  std::vector<dvs_msgs::Event> batch_events;
  batch_events.resize(batch_size);

  float w0, w1, w2, w3;

  cv::Point2f p_ref;
  cv::Point2f tl;
  cv::Vec3f p_norm_cur, p_ref_homogeneous;
  p_norm_cur[2] = 1.f;

  while (e != ev_last) {
    batch_events[index_batch] = *e;
    if (index_batch >= batch_size - 1 || e == ev_last - 1) {
      // Fix for the remaining events in the last batch.
      if (index_batch < batch_size - 1) {
        batch_events.erase(batch_events.begin() + index_batch,
                           batch_events.end());
      }

      const double t = batch_events[0].ts.toSec() - t0;

      // precompute the current instantaneous homography matrix, valid for the
      // whole batch cv::Matx31f T = v * t;
      cv::Matx33f R = I3 + Sw * t;

      // H = R + T * n^T
      cv::Matx33f H_cur_begin = R;

      cv::Matx33f H_begin_cur = H_cur_begin.inv();
      cv::Matx33f H_ref_cur = H_ref_begin * H_begin_cur;
      cv::Matx33f KH_ref_cur = K_refx_ * H_ref_cur;

      // process the batch
      for (const dvs_msgs::Event &ev : batch_events) {
        const cv::Point2f &p = rectified_points_[ev.x + ev.y * sensor_size_.width];
        p_norm_cur[0] = p.x;
        p_norm_cur[1] = p.y;
        p_ref_homogeneous = KH_ref_cur * p_norm_cur;

        p_ref.x = p_ref_homogeneous[0] / p_ref_homogeneous[2];
        p_ref.y = p_ref_homogeneous[1] / p_ref_homogeneous[2];

        tl.x = int_floor(p_ref.x);
        tl.y = int_floor(p_ref.y);

        if (rect_.contains(tl)) {
          const float fx = p_ref.x - tl.x, fy = p_ref.y - tl.y;

          //w0 = (1.f - fx) * (1.f - fy);
          //w1 = (fx) * (1.f - fy);
          //w2 = (1.f - fx) * (fy);
          //w3 = (fx) * (fy);

          float rel_ts = (float)(e->ts - ev_first->ts).toSec();
          normalized_mean_timestamp_image_.at<float>(tl.y, tl.x) += rel_ts;
          //normalized_mean_timestamp_image_.at<float>(tl.y, tl.x + 1) += rel_ts * w1;
          //normalized_mean_timestamp_image_.at<float>(tl.y + 1, tl.x) += rel_ts * w2;
          //normalized_mean_timestamp_image_.at<float>(tl.y + 1, tl.x + 1) += rel_ts * w3;

          event_count.at<float>(tl.y, tl.x) += 1;
          //event_count.at<float>(tl.y, tl.x + 1) += w1;
          //event_count.at<float>(tl.y + 1, tl.x) += w2;
          //event_count.at<float>(tl.y + 1, tl.x + 1) += w3;
        }
      }

      // reset the batch
      index_batch = 0;
    } else {
      ++index_batch;
    }
    ++e;
  }

  float mean_rel_ts = 0.f;
  int count_rel_ts = 0;
  for (int y = 0; y < normalized_mean_timestamp_image_.rows; ++y) {
    for (int x = 0; x < normalized_mean_timestamp_image_.cols; ++x) {
      if (event_count.at<float>(y, x) > 0) {
        normalized_mean_timestamp_image_.at<float>(y, x) =
            normalized_mean_timestamp_image_.at<float>(y, x) /
            event_count.at<float>(y, x);
        mean_rel_ts += normalized_mean_timestamp_image_.at<float>(y, x);
        count_rel_ts += 1;
      } else {
        normalized_mean_timestamp_image_.at<float>(y, x) = 0.;
      }
    }
  }
  if (count_rel_ts > 0)
    mean_rel_ts /= (float)count_rel_ts;

  float deltaT = (ev_last->ts - ev_first->ts).toSec();
  normalized_mean_timestamp_image_ =
      (normalized_mean_timestamp_image_ - mean_rel_ts) / deltaT;
  most_recent_event_ts_ = ev_last->ts;
}

void NormalizedMeanTimestampImage::filterImage() {
  if (opening_iterations_ > 0)
    cv::morphologyEx(thresholded_mean_timestamp_image_,
                     thresholded_mean_timestamp_image_, cv::MORPH_OPEN,
                     element_open_, cv::Point(-1, -1), opening_iterations_,
                     cv::BORDER_CONSTANT, 0);
  if (closing_iterations_ > 0)
    cv::morphologyEx(thresholded_mean_timestamp_image_,
                     thresholded_mean_timestamp_image_, cv::MORPH_CLOSE,
                     element_close_, cv::Point(-1, -1), closing_iterations_,
                     cv::BORDER_CONSTANT, 0);
  if (dilate_iterations_ > 0) {
    cv::dilate(thresholded_mean_timestamp_image_,
               thresholded_mean_timestamp_image_, dilate_kernel_,
               cv::Point(-1, -1), dilate_iterations_);
  }
}

void NormalizedMeanTimestampImage::meanFilterImage() {
  if (raw_image_filter_) {
    sepFilter2D(normalized_mean_timestamp_image_,
                normalized_mean_timestamp_image_, -1,
                raw_image_filter_kernel_r_, raw_image_filter_kernel_c_);
  }
}

void NormalizedMeanTimestampImage::loadParameters() {
  thresh_a_ = (float)rpg_dynamic_obstacle_detection::param<double>(
      pnh_, "normalized_mean_timestamp_image/thresh_a", 0.0);
  thresh_b_ = (float)rpg_dynamic_obstacle_detection::param<double>(
      pnh_, "normalized_mean_timestamp_image/thresh_b", 0.04);

  raw_image_filter_ =
      param<int>(pnh_, "timestamp_image_filter/raw_image_filter", 1);

  int morphology_kernel_size =
      param<int>(pnh_, "timestamp_image_filter/morphology_kernel_size_open", 5);
  element_open_ = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(morphology_kernel_size, morphology_kernel_size));
  morphology_kernel_size = param<int>(
      pnh_, "timestamp_image_filter/morphology_kernel_size_close", 5);
  element_close_ = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(morphology_kernel_size, morphology_kernel_size));
  opening_iterations_ =
      param<int>(pnh_, "timestamp_image_filter/opening_iterations", 1);
  closing_iterations_ =
      param<int>(pnh_, "timestamp_image_filter/closing_iterations", 1);
  dilate_iterations_ =
      param<int>(pnh_, "timestamp_image_filter/dilate_iterations", 1);
  morphology_kernel_size = param<int>(
      pnh_, "timestamp_image_filter/morphology_kernel_size_dilate", 5);
  dilate_kernel_ = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(morphology_kernel_size, morphology_kernel_size));
}
} // namespace rpg_dynamic_obstacle_detection
