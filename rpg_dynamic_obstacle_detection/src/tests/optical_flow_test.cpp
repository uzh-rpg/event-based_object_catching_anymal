#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "rpg_dynamic_obstacle_detection/optical_flow.hpp"
#include "rpg_dynamic_obstacle_detection_msgs/OpticalFlowImages.h"

#include "rpg_dynamic_obstacle_detection/timer.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "OF_test");
  ros::NodeHandle nh;
  Timer timer;

  rpg_dynamic_obstacle_detection::OpticalFlow optical_flow;

  cv::Mat img1(50, 50, CV_8UC1);
  cv::Mat img2(50, 50, CV_8UC1);
  img1.setTo(0);
  img2.setTo(0);

  cv::Rect rect1(20, 20, 14, 14);
  cv::Rect rect11(25, 25, 4, 4);
  cv::rectangle(img1, rect1, cv::Scalar(255), 1);
  cv::rectangle(img1, rect11, cv::Scalar(255), 1);
  cv::Rect rect2(21, 20, 14, 14);
  cv::Rect rect22(25, 26, 4, 4);
  cv::rectangle(img2, rect2, cv::Scalar(255), 1);
  cv::rectangle(img2, rect22, cv::Scalar(255), 1);

  // cv::circle(img1, cv::Point(20,20), 5, cv::Scalar(255), 2);
  // cv::circle(img2, cv::Point(25,20), 5, cv::Scalar(255), 2);

  cv::imshow("img1", img1);
  cv::waitKey(1);
  cv::imshow("img2", img2);
  cv::waitKey(1);

  cv::Mat u = cv::Mat::zeros(img1.rows, img1.cols, CV_32FC1);
  cv::Mat v = cv::Mat::zeros(img1.rows, img1.cols, CV_32FC1);
  optical_flow.getLucasKanadeOpticalFlow(img1, u, v);
  u.setTo(0.0);
  v.setTo(0.0);

  timer.start();
  optical_flow.getLucasKanadeOpticalFlow(img2, u, v);
  timer.stop();
  std::cout << "Calc Time OF: " << timer.getMilliseconds() << std::endl;

  timer.reset();
  timer.start();
  cv::medianBlur(u, u, 3);
  cv::medianBlur(v, v, 3);
  timer.stop();
  std::cout << "Calc Time Median: " << timer.getMilliseconds() << std::endl;

  ros::Publisher image_pub =
      nh.advertise<rpg_dynamic_obstacle_detection_msgs::OpticalFlowImages>(
          "debug_image", 100);

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  rpg_dynamic_obstacle_detection_msgs::OpticalFlowImages of_images;
  cv_bridge::CvImage debug_image;
  debug_image.header.stamp = ros::Time::now();
  debug_image.encoding = sensor_msgs::image_encodings::MONO8;
  debug_image.image = img2;
  debug_image.toImageMsg(of_images.image);
  debug_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  debug_image.image = u;
  debug_image.toImageMsg(of_images.u);
  debug_image.image = v;
  debug_image.toImageMsg(of_images.v);

  image_pub.publish(of_images);
  ros::spinOnce();

  std::cout << "Completed" << std::endl;

  ros::waitForShutdown();

  return 0;
}
