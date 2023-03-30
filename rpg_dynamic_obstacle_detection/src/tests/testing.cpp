#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include <iostream>
#include <math.h>

#include "rpg_dynamic_obstacle_detection/average_timer.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ellipse_detector");

  /*
  cv::Mat src = cv::Mat::ones(40,40, CV_32FC1);
  cv::Mat kernel = cv::Mat::ones(11, 11, CV_32FC1);
  cv::Mat dst;

  filter2D(src, dst, -1, kernel, cv::Point(-1, -1));

  std::cout << src << std::endl;
  std::cout << "----------------------------" << std::endl;
  std::cout << dst << std::endl;
*/

  std::vector<int> v(5);
  v[0] = 1;
  std::cout << v.back() << std::endl;
  std::cout << v.size() << std::endl;
  return 0;
}
