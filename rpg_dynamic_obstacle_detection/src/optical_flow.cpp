#include "rpg_dynamic_obstacle_detection/optical_flow.hpp"

namespace rpg_dynamic_obstacle_detection {
OpticalFlow::OpticalFlow() : initialized_(false), last_frame_empty_(true) {
  timer_ = AverageTimer("OpticalFlow", 10000);
}

bool OpticalFlow::getLucasKanadeOpticalFlow(const cv::Mat &frame_new,
                                            cv::Mat &u, cv::Mat &v,
                                            const cv::Mat &thresh_image) {
  if (frame_new.cols == 0)
    return false;
  if (!initialized_) {
    frame_prev_ = cv::Mat::zeros(frame_new.rows, frame_new.cols, CV_32FC1);
    initialized_ = true;
  }
  if (cv::countNonZero(thresh_image) == 0) {
    frame_prev_.setTo(0.0);
    if (u.empty() || v.empty()) {
      u = cv::Mat::zeros(frame_new.rows, frame_new.cols, CV_32FC1);
      v = cv::Mat::zeros(frame_new.rows, frame_new.cols, CV_32FC1);
    } else {
      u.setTo(0.0);
      v.setTo(0.0);
    }
    return false;
  }
  // timer_.restart();
  frame_new.convertTo(frame_new_, CV_32FC1, 1.0 / 255.0, 0);
  if (mean_input_filter_width_ > 1)
    meanWindowFilter(frame_new_, mean_input_filter_width_);
  lucasKanadeOpticalFlow(frame_prev_, frame_new_, u, v);
  if (median_filter_width_ > 0) {
    cv::medianBlur(u, u, median_filter_width_);
    cv::medianBlur(v, v, median_filter_width_);
  }
  if (mean_output_filter_width_ > 1) {
    meanWindowFilter(u, mean_output_filter_width_);
    meanWindowFilter(v, mean_output_filter_width_);
  }
  // TODO: can one use std::swap here?
  frame_prev_ = frame_new_.clone();
  // timer_.stop();
  // LOG_EVERY_N(INFO, 5) << timer_.get_info_string();
  return true;
}

bool OpticalFlow::getLucasKanadeOpticalFlowReduced(
    const cv::Mat &frame_new, cv::Mat &u, cv::Mat &v,
    const cv::Mat &thresh_image) {

  if (!initialized_) {
    frame_prev_ = cv::Mat::zeros(frame_new.rows, frame_new.cols, CV_32FC1);
    initialized_ = true;
  
  }
  if (!inclusionRect(thresh_image)) {
    frame_prev_.setTo(0.0);
    if (u.empty() || v.empty()) {
      u = cv::Mat::zeros(frame_new.rows, frame_new.cols, CV_32FC1);
      v = cv::Mat::zeros(frame_new.rows, frame_new.cols, CV_32FC1);
    } else {
      u.setTo(0.0);
      v.setTo(0.0);
    }
  
    return false;
  }
  // timer_.restart();

  frame_new.convertTo(frame_new_, CV_32FC1, 1.0 / 255.0, 0);
  if (mean_input_filter_width_ > 1)
    meanWindowFilter(frame_new_, mean_input_filter_width_);
  cv::Mat u_tmp;
  cv::Mat v_tmp;

  lucasKanadeOpticalFlow(frame_prev_(inclusion_rect_),
                         frame_new_(inclusion_rect_), u_tmp, v_tmp);

  if (median_filter_width_ > 0) {
    cv::medianBlur(u_tmp, u_tmp, median_filter_width_);
    cv::medianBlur(v_tmp, v_tmp, median_filter_width_);
  }

  if (mean_output_filter_width_ > 1) {
    meanWindowFilter(u_tmp, mean_output_filter_width_);
    meanWindowFilter(v_tmp, mean_output_filter_width_);
  }

  u_tmp.copyTo(u(inclusion_rect_));
  v_tmp.copyTo(v(inclusion_rect_));
  frame_prev_ = frame_new_.clone();

  // timer_.stop();
  // LOG_EVERY_N(INFO, 5) << timer_.get_info_string();
  return true;
}

bool OpticalFlow::getLucasKanadeOpticalFlow(const cv::Mat &frame_new,
                                            cv::Mat &u, cv::Mat &v) {
  return getLucasKanadeOpticalFlow(frame_new, u, v, frame_new);
}

bool OpticalFlow::containsRect(const cv::Mat &mat, const cv::Rect &rect) {
  return (rect & cv::Rect(0, 0, mat.cols, mat.rows)) == rect;
}

cv::Mat OpticalFlow::getFx(const cv::Mat &src1, const cv::Mat &src2) {
  cv::Mat fx;
  cv::Mat dst1, dst2;
  /*
  filter2D(src1, dst1, -1, kernel_fx_);
  filter2D(src2, dst2, -1, kernel_fx_);
  */
  sepFilter2D(src1, dst1, -1, kernel_fx_r_, kernel_fx_c_);
  sepFilter2D(src2, dst2, -1, kernel_fx_r_, kernel_fx_c_);

  fx = dst1 + dst2;
  return fx;
}

cv::Mat OpticalFlow::getFy(const cv::Mat &src1, const cv::Mat &src2) {
  cv::Mat fy;
  cv::Mat dst1, dst2;
  /*
  filter2D(src1, dst1, -1, kernel_fy_);
  filter2D(src2, dst2, -1, kernel_fy_);
  */
  sepFilter2D(src1, dst1, -1, kernel_fy_r_, kernel_fy_c_);
  sepFilter2D(src2, dst2, -1, kernel_fy_r_, kernel_fy_c_);

  fy = dst1 + dst2;
  return fy;
}

cv::Mat OpticalFlow::getFt(const cv::Mat &src1, const cv::Mat &src2) {
  cv::Mat ft;
  cv::Mat dst1, dst2;
  /*
  filter2D(src1, dst1, -1, kernel_ft1_);
  filter2D(src2, dst2, -1, kernel_ft2_);
  */
  sepFilter2D(src1, dst1, -1, kernel_ft1_r_, kernel_ft1_c_);
  sepFilter2D(src2, dst2, -1, kernel_ft2_r_, kernel_ft2_c_);

  ft = dst1 + dst2;
  return ft;
}

cv::Mat OpticalFlow::getWindowSumMat(const cv::Mat &m, const size_t &wwh) {
  /*
  cv::Mat kernel_r = cv::Mat::ones(1, 2*wwh+1, CV_32FC1);
  cv::Mat kernel_c = cv::Mat::ones(2*wwh+1, 1, CV_32FC1);
  // cv::Mat kernel = cv::Mat::ones(2*wwh+1, 2*wwh+1, CV_32FC1);
  cv::Mat res;
  //filter2D(m, res, -1, kernel, cv::Point(-1, -1));
  sepFilter2D(m, res, -1, kernel_r, kernel_c);
  */
  cv::Mat area = cv::Mat::zeros(m.rows, m.cols, CV_32FC1);
  cv::Mat res = cv::Mat::zeros(m.rows, m.cols, CV_32FC1);
  for (size_t y = 0; y < m.rows; ++y) {
    float sum = 0.0;
    if (y == 0) {
      for (size_t x = 0; x < m.cols; ++x) {
        sum += m.at<float>(y, x);
        area.at<float>(y, x) = sum;
      }
    } else {
      for (size_t x = 0; x < m.cols; ++x) {
        sum += m.at<float>(y, x);
        area.at<float>(y, x) = sum + area.at<float>(y - 1, x);
      }
    }
  }
  for (size_t y = wwh; y < m.rows - wwh; ++y) {
    for (size_t x = wwh; x < m.cols - wwh; ++x) {
      res.at<float>(y, x) =
          area.at<float>(y + wwh, x + wwh) - area.at<float>(y + wwh, x - wwh) -
          area.at<float>(y - wwh, x + wwh) + area.at<float>(y - wwh, x - wwh);
    }
  }
  // TODO: Border area is not considered yet.
  return res;
}

void OpticalFlow::lucasKanadeOpticalFlow(const cv::Mat &img1,
                                         const cv::Mat &img2, cv::Mat &u,
                                         cv::Mat &v) {
  cv::Mat fx = getFx(img1, img2);
  cv::Mat fy = getFy(img1, img2);
  cv::Mat ft = getFt(img1, img2);

  cv::Mat sumfx2 = getWindowSumMat(fx.mul(fx), wwh_);
  cv::Mat sumfy2 = getWindowSumMat(fy.mul(fy), wwh_);
  cv::Mat sumfxft = getWindowSumMat(fx.mul(ft), wwh_);
  cv::Mat sumfxfy = getWindowSumMat(fx.mul(fy), wwh_);
  cv::Mat sumfyft = getWindowSumMat(fy.mul(ft), wwh_);

  cv::Mat tmp = sumfx2.mul(sumfy2) - sumfxfy.mul(sumfxfy);
  u = sumfxfy.mul(sumfyft) - sumfy2.mul(sumfxft);
  v = sumfxft.mul(sumfxfy) - sumfx2.mul(sumfyft);
  cv::divide(u, tmp, u);
  cv::divide(v, tmp, v);
}

bool OpticalFlow::inclusionRect(const cv::Mat &input) {
  if (inc_std_ == 0.0) {
    inclusion_rect_ = cv::Rect(0, 0, input.cols, input.rows);
    if (cv::countNonZero(input) == 0)
      return false;
  } else {
    if (cv::countNonZero(input) > 0) {
      std::vector<cv::Point> nonzero;
      cv::Scalar nz_mean, nz_std;
      cv::findNonZero(input, nonzero);
      cv::meanStdDev(nonzero, nz_mean, nz_std);
      cv::Point2f mean_new(nz_mean[0], nz_mean[1]);
      cv::Point2f std_new(nz_std[0], nz_std[1]);

      size_t min_row, min_col, max_row, max_col;
      if (last_frame_empty_) {
        min_row =
            std::max((int)floor(mean_new.y - inc_std_ * std_new.y) - wwh_, 0);
        min_col =
            std::max((int)floor(mean_new.x - inc_std_ * std_new.x) - wwh_, 0);
        max_row = std::min((int)ceil(mean_new.y + inc_std_ * std_new.y) + wwh_,
                           input.rows);
        max_col = std::min((int)ceil(mean_new.x + inc_std_ * std_new.x) + wwh_,
                           input.cols);
      } else {
        min_row = std::max(
            (int)floor(std::min(mean_new.y - inc_std_ * std_new.y,
                                mean_prev_.y - inc_std_ * std_prev_.y)) -
                wwh_,
            0);
        min_col = std::max(
            (int)floor(std::min(mean_new.x - inc_std_ * std_new.x,
                                mean_prev_.x - inc_std_ * std_prev_.x)) -
                wwh_,
            0);
        max_row = std::min(
            (int)ceil(std::max(mean_new.y + inc_std_ * std_new.y,
                               mean_prev_.y + inc_std_ * std_prev_.y)) +
                wwh_,
            input.rows);
        max_col = std::min(
            (int)ceil(std::max(mean_new.x + inc_std_ * std_new.x,
                               mean_prev_.x + inc_std_ * std_prev_.x)) +
                wwh_,
            input.cols);
      }
      inclusion_rect_ =
          cv::Rect(min_col, min_row, max_col - min_col, max_row - min_row);
      mean_prev_ = mean_new;
      std_prev_ = std_new;
      last_frame_empty_ = false;
    } else {
      last_frame_empty_ = true;
      return false;
    }
  }
  return true;
}

void OpticalFlow::meanWindowFilter(cv::Mat &src, const size_t &window_width) {
  src = getWindowSumMat(src, (size_t)floor((float)window_width / 2.0));
  src = src / (float)(window_width * window_width);
}

void OpticalFlow::initialize(size_t window_width, double inc_std = 0.0,
                             size_t median_filter_width = 0,
                             size_t mean_input_filter_width = 0,
                             size_t mean_output_filter_width = 0) {
  window_width_ = window_width;
  if (window_width_ % 2 == 0)
    LOG(ERROR) << "optical_flow_window_width is not an odd number: "
               << window_width_;
  wwh_ = (uint8_t)floor((float)window_width_ / 2.0);
  inc_std_ = inc_std;
  median_filter_width_ = median_filter_width; 
  mean_input_filter_width_ = mean_input_filter_width;
  mean_output_filter_width_ = mean_output_filter_width;

  /*
  kernel_fx_ = cv::Mat::ones(2, 2, CV_32FC1);
  kernel_fx_.at<float>(0, 0) = -1.0;
  kernel_fx_.at<float>(1, 0) = -1.0;

  kernel_fy_ = cv::Mat::ones(2, 2, CV_32FC1);
  kernel_fy_.at<float>(0, 0) = -1.0;
  kernel_fy_.at<float>(0, 1) = -1.0;

  kernel_ft1_ = cv::Mat::ones(2, 2, CV_32FC1);
  kernel_ft1_ = kernel_ft1_.mul(-1);
  kernel_ft2_ = kernel_ft1_.mul(-1);
  */

  kernel_fx_r_ = cv::Mat::ones(1, 2, CV_32FC1);
  kernel_fx_r_.at<float>(0, 0) = -1.0;
  kernel_fx_c_ = cv::Mat::ones(2, 1, CV_32FC1);

  kernel_fy_r_ = cv::Mat::ones(1, 2, CV_32FC1);
  kernel_fy_r_.at<float>(0, 0) = -1.0;
  kernel_fy_c_ = cv::Mat::ones(2, 1, CV_32FC1);
  kernel_fy_c_.at<float>(1, 0) = -1.0;

  kernel_ft1_r_ = cv::Mat::ones(1, 2, CV_32FC1);
  kernel_ft1_r_ = kernel_ft1_r_.mul(-1.0);
  kernel_ft1_c_ = cv::Mat::ones(2, 1, CV_32FC1);

  kernel_ft2_r_ = cv::Mat::ones(1, 2, CV_32FC1);
  kernel_ft2_c_ = cv::Mat::ones(2, 1, CV_32FC1);
}
} // namespace rpg_dynamic_obstacle_detection
