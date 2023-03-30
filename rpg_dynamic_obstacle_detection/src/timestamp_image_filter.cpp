#include "rpg_dynamic_obstacle_detection/timestamp_image_filter.hpp"

namespace rpg_dynamic_obstacle_detection {

TimestampImageFilter::TimestampImageFilter(ros::NodeHandle nh,
                                           ros::NodeHandle pnh,
                                           const cv::Mat &input,
                                           std::mutex &timespamp_image_mutex)
    : nh_(nh), pnh_(pnh), input_(input),
      timespamp_image_mutex_(timespamp_image_mutex) {
  loadParameters();
}

void TimestampImageFilter::filterImage() {
  timespamp_image_mutex_.lock();
  if (input_.rows > 0 && input_.cols > 0) {
    cv::morphologyEx(input_, output_, cv::MORPH_OPEN, element_open_,
                     cv::Point(-1, -1), opening_iterations_,
                     cv::BORDER_CONSTANT, 0);
    cv::morphologyEx(output_, output_, cv::MORPH_CLOSE, element_close_,
                     cv::Point(-1, -1), closing_iterations_,
                     cv::BORDER_CONSTANT, 0);
    // cv::blur(output_, output_, cv::Size(3, 3));
    // cv::threshold(output_, output_, 90, 255, cv::THRESH_BINARY);
  }
  timespamp_image_mutex_.unlock();
}

void TimestampImageFilter::loadParameters() {
  int morphology_kernel_size =
      param<int>(pnh_, "morphology_kernel_size_open", 5);
  element_open_ = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(morphology_kernel_size, morphology_kernel_size));
  morphology_kernel_size = param<int>(pnh_, "morphology_kernel_size_close", 5);

  element_close_ = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(morphology_kernel_size, morphology_kernel_size));
  opening_iterations_ = param<int>(pnh_, "opening_iterations", 1);
  closing_iterations_ = param<int>(pnh_, "closing_iterations", 1);
}
} // namespace rpg_dynamic_obstacle_detection
