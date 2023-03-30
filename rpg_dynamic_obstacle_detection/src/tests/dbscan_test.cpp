#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include "clustering/dbscan.h"

#include "rpg_dynamic_obstacle_detection/timer.h"

template <typename T> inline std::vector<T> minSet(std::vector<T> vec) {
  std::vector<T> min_set;
  for (T v : vec) {
    if (!(std::find(min_set.begin(), min_set.end(), v) != min_set.end())) {
      min_set.push_back(v);
    }
  }
  return min_set;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dbscan_test");
  ros::NodeHandle nh;
  Timer timer;

  cv::Mat img(50, 50, CV_8UC1);
  img.setTo(0);

  cv::Rect rect1(2, 2, 14, 14);
  cv::Rect rect2(30, 30, 14, 14);
  cv::rectangle(img, rect1, cv::Scalar(255), 3);
  cv::rectangle(img, rect2, cv::Scalar(255), 3);

  // cv::circle(img1, cv::Point(20,20), 5, cv::Scalar(255), 2);

  cv::imshow("img", img);
  cv::waitKey(1);

  std::vector<cv::Point> nonzero;
  cv::findNonZero(img, nonzero);
  ublas::matrix<double> cluster_data(nonzero.size(), 5);
  for (size_t i = 0; i < nonzero.size(); ++i) {
    cluster_data(i, 0) = (double)nonzero[i].x;
    cluster_data(i, 1) = (double)nonzero[i].y;
    cluster_data(i, 2) = 0.0;
    cluster_data(i, 3) = 0.0;
    cluster_data(i, 4) = 0.0;
  }
  ublas::vector<double> features_weights(3);
  features_weights(0) = 1.0;
  features_weights(1) = 0.0;
  features_weights(2) = 0.0;

  clustering::DBSCAN dbscan(0.1, 5, 1);
  dbscan.reset();
  dbscan.wfit(cluster_data, features_weights);

  std::vector<int32_t> label_set = minSet<int32_t>(dbscan.get_labels());

  std::cout << "Label Set:" << std::endl;
  for (auto i : label_set) {
    std::cout << i << std::endl;
  }

  ros::waitForShutdown();

  return 0;
}
