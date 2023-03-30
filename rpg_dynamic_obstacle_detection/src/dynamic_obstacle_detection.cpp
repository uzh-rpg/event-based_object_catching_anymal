#include "rpg_dynamic_obstacle_detection/dynamic_obstacle_detection.hpp"
#include <typeinfo>
#include <fstream>

namespace rpg_dynamic_obstacle_detection {

DynamicObstacleDetection::DynamicObstacleDetection(ros::NodeHandle nh,
                                                   ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh), dvs_handler_(nh_, pnh_),
      imu_warp_(nh_, pnh_, dvs_handler_.getEventQueue(),
                dvs_handler_.getFrameBegin(), dvs_handler_.getFrameEnd()),
      nmti_(nh_, pnh_, dvs_handler_.getEventQueue(),
            dvs_handler_.getRectifiedPoints(), imu_warp_.getWarp(),
            dvs_handler_.getFrameBegin(), dvs_handler_.getFrameEnd(),
            dvs_handler_.getKRefx(), dvs_handler_.getSensorSize())
           {

  file_handle_.open("/tmp/detection.txt");

  loadParameters();

  timer_1_ = AverageTimer("NMTI", 100000);
  timer_2_ = AverageTimer("vel", 100000);

  reconfigure_srv_.setCallback(boost::bind(
      &DynamicObstacleDetection::reconfigureCallback, this, _1, _2));


  dbscan_.init(dbscan_eps_, dbscan_features_weights_, dbscan_use_rotated_rect_);
  dbscan_.initOpticalFlow(of_window_width_, of_inc_std_,
                          of_median_filter_width_, of_mean_input_filter_width_,
                          of_mean_output_filter_width_);

  if (use_optimized_camera_matrix_) {
    cv::cv2eigen(dvs_handler_.getCameraMatrix(), camera_mat_);
  }
  LOG(INFO) << "camera_mat:\n" << camera_mat_;
  focal_length_ = (camera_mat_(0, 0) + camera_mat_(1, 1)) / 2.0;
  LOG(INFO) << "Focal length: " << focal_length_;
  Eigen::Matrix4d T_BC = Eigen::Matrix4d::Identity();

  DynamicObstacleDetection::initializeImage2World(
      camera_mat_, focal_length_, width_real_, depth_uncertanty_, depth_object_,
      T_BC, depth_limit_);
  
  cluster_center_pub_ = 
      nh_.advertise<geometry_msgs::PointStamped>("cluster_bbox",1);
  imu_warp_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("imu_warp", 1);
  update_timer_ = nh_.createTimer(ros::Rate(update_frequency_),
                                  &DynamicObstacleDetection::updateLoop, this);

  image_process_lock_.lock();
  image_process_ =
      std::thread(&DynamicObstacleDetection::imageProcessing, this);
  image_process_.detach();


  // debug
  image_pub_ = nh_.advertise<sensor_msgs::Image>("debug_image", 100);
  image_cluster_pub_ = nh_.advertise<sensor_msgs::Image>("cluster_image", 100);
  average_timestamp_pub_ =
      nh_.advertise<sensor_msgs::Image>("average_timestamp", 100);
  optical_flow_pub_ =
      nh_.advertise<rpg_dynamic_obstacle_detection_msgs::OpticalFlowImages>(
          "optical_flow_image", 100);
}

void DynamicObstacleDetection::reconfigureCallback(
    DynamicObstacleDetectionConfig &config, uint32_t level) {
  if (!reconfigure_was_called_before_) {
    reconfigure_was_called_before_ = true;
    return;
  }

  if (level == 1) {
    rectangle_ratio_limit_ = config.rectangle_ratio_limit;
    ROS_WARN("[%s] Changed rectangle_ratio_limit to: %.5f",
             ros::this_node::getName().c_str(), rectangle_ratio_limit_);
  }
  if (level == 2) {
    double a = config.thresh_a;
    double b = config.thresh_b;
    nmti_.changeThreshold(a, b);
    ROS_WARN("[%s] Changed thresh_a to: %.5f",
             ros::this_node::getName().c_str(), a);
    ROS_WARN("[%s] Changed thresh_b to: %.5f",
             ros::this_node::getName().c_str(), b);
  }
  if (level == 3) {
    nmti_.changeFilterRaw(config.groups.timestampimage.raw_image_filter);
    nmti_.changeFilterOpen(
        config.groups.timestampimage.morphology_opening_iterations,
        config.groups.timestampimage.morphology_kernel_size_open);
    nmti_.changeFilterClose(
        config.groups.timestampimage.morphology_closing_iterations,
        config.groups.timestampimage.morphology_kernel_size_close);
    nmti_.changeFilterDilate(
        config.groups.timestampimage.morphology_dilate_iterations,
        config.groups.timestampimage.morphology_kernel_size_dilate);
    ROS_WARN("Changed Filter");
  }
  if (level == 4) {
    dbscan_.initOpticalFlow(
        config.groups.opticalflow.window_width,
        config.groups.opticalflow.inc_std,
        config.groups.opticalflow.median_output_filter_width,
        config.groups.opticalflow.mean_input_filter_width,
        config.groups.opticalflow.mean_output_filter_width);
    ROS_WARN("Changed OpticalFlow");
  }
  if (level == 5) {
    dbscan_eps_ = config.groups.cluster.eps;
    dbscan_features_weights_ = {config.groups.cluster.weight_pos,
                                config.groups.cluster.weight_vel,
                                config.groups.cluster.weight_timestamp};
    dbscan_use_rotated_rect_ = config.groups.cluster.use_rotated_rect;
    dbscan_.init(dbscan_eps_, dbscan_features_weights_,
                 dbscan_use_rotated_rect_);
    ROS_WARN("Changed DBSCAN");
  }
}

void DynamicObstacleDetection::imageProcessing() {
  cv::Mat img, thresh_image;
  while (ros::ok()) {
    image_process_lock_.lock();

  nmti_.lock();
  nmti_.getNormalizedMeanTimestampImage().copyTo(img);
  nmti_.getThresholdedMeanTimestampImage().copyTo(thresh_image);
  ros::Time ros_time = nmti_.getMostRecentTime();
  nmti_.unlock();

    geometry_msgs::TwistStamped imu_warp_msg;
    imu_warp_msg.header.stamp = ros_time;
    cv::Mat warp = imu_warp_.getWarp();
    imu_warp_msg.twist.angular.x = warp.at<double>(0);
    imu_warp_msg.twist.angular.y = warp.at<double>(1);
    imu_warp_msg.twist.angular.z = warp.at<double>(2);
    
    double time = ros_time.toSec();

    dbscan_.cluster(img, thresh_image);
    double max_area = 0;
    cv::RotatedRect max_rect;
    for (const auto &r : dbscan_.getClusterRects()) {
      double rect_area = r.size.width * r.size.height;
      // std::cout << "rect_area: " << rect_area << std::endl;
      if (rect_area > max_area) {
        max_area = rect_area;
        max_rect = r;
      }
    }
    cv::Point2f pts[4];
    max_rect.points(pts);
    std::vector<Eigen::Vector2d> corner_points;
    corner_points.reserve(4);
    for (size_t i = 0; i < 4; ++i) {
      corner_points.emplace_back(pts[i].x, pts[i].y);
    }

    double size_measured, height;
    // std::cout << "use_min_or_max_side_ =" << use_min_or_max_side_ << std::endl;
    // std::cout << "max_rect.size.width =" << max_rect.size.width << std::endl;
    // std::cout << "max_rect.size.height =" << max_rect.size.height << std::endl;
    if (use_min_or_max_side_ == 1) {
      size_measured = std::max(max_rect.size.width, max_rect.size.height);
      height = std::min(max_rect.size.width, max_rect.size.height);
    } else if (use_min_or_max_side_ == -1) {
      size_measured = std::min(max_rect.size.width, max_rect.size.height);
      height = std::max(max_rect.size.width, max_rect.size.height);
    }
    // std::cout << "size_measured: " << size_measured << std::endl;
    // std::cout << "height: " << height << std::endl;
    if (height > 16) {
      // calculate current x, y
      position_x = max_rect.center.x;
      position_y = max_rect.center.y;
      position_z = 0.0;
      // backproject the current detection
      Eigen::Vector3d world_point;
      Eigen::Vector2d image_point;
      image_point << position_x, position_y;
      DynamicObstacleDetection::project2base(world_point, height, image_point);
      position_x = world_point[0];
      position_y = world_point[1];
      position_z = world_point[2];
      }
    else
      {
        position_x = 0.0;
        position_y = 0.0;
        position_z = -1.0;
      }

    geometry_msgs::PointStamped stamped_position_3d;
    stamped_position_3d.header.stamp = ros_time;
    stamped_position_3d.point.x = position_x;
    stamped_position_3d.point.y = position_y;
    stamped_position_3d.point.z = position_z;

    if (cluster_center_pub_.getNumSubscribers() > 0) {
      cluster_center_pub_.publish(stamped_position_3d);
    }

    if (imu_warp_pub_.getNumSubscribers() > 0) {
      imu_warp_pub_.publish(imu_warp_msg);
    }

    if (image_pub_.getNumSubscribers() > 0) {
      cv::cvtColor(thresh_image, debug_image_, cv::COLOR_GRAY2RGB);
    }

    if (average_timestamp_pub_.getNumSubscribers() > 0) {
      cv::Mat img_color;
      img.convertTo(img_color, CV_8UC3, 255);
      cv::applyColorMap(img_color, img_color, cv::COLORMAP_HOT);
      cv::Mat img_color_flipped;
      cv::flip(img_color, img_color_flipped, -1);
      cv_bridge::CvImage debug_image;
      debug_image.header.stamp = ros_time;
      debug_image.encoding = sensor_msgs::image_encodings::BGR8;
      debug_image.image = img_color_flipped;
      average_timestamp_pub_.publish(debug_image.toImageMsg());
    }

    if (optical_flow_pub_.getNumSubscribers() > 0) {
      rpg_dynamic_obstacle_detection_msgs::OpticalFlowImages of_images;
      cv_bridge::CvImage debug_image;
      debug_image.header.stamp = ros_time;
      debug_image.encoding = sensor_msgs::image_encodings::MONO8;
      img.convertTo(debug_image.image, CV_8UC1, 255);
      debug_image.toImageMsg(of_images.image);
      debug_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      debug_image.image = dbscan_.getOpticalFlowU();
      debug_image.toImageMsg(of_images.u);
      debug_image.image = dbscan_.getOpticalFlowV();
      debug_image.toImageMsg(of_images.v);
      optical_flow_pub_.publish(of_images);
    }

    if (image_cluster_pub_.getNumSubscribers() > 0) {
      cv::cvtColor(thresh_image, debug_image_, cv::COLOR_GRAY2RGB);

      static int image_counter = 0;

      cv::Mat rgb = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
      cv::cvtColor(thresh_image, rgb, cv::COLOR_GRAY2RGB);
      for (int i=0; i<dbscan_.getClusterRects().size(); i++) {
          auto &r = dbscan_.getClusterRects()[i];
          cv::Point2f rect_points[4];
          r.points(rect_points);
          for (size_t j = 0; j < 4; j++) {
            line(rgb, rect_points[j], rect_points[(j + 1) % 4],
            cv::Scalar(255, 0, 0), 1);
          }
          image_counter++;
      }

      cv::RNG rng(12345);
      for (const auto &cluster : dbscan_.getClusterData()) {
        cv::Point3_<uchar> color(rng.uniform(0, 255), rng.uniform(0, 255),
                                 rng.uniform(0, 255));
        for (const auto &p : cluster) {
          rgb.at<cv::Point3_<uchar>>(p) = color;
        }
      }
      cv::Mat rgb_flipped;
      cv::flip(rgb, rgb_flipped, -1);
      cv_bridge::CvImage debug_image;
      debug_image.header.stamp = ros_time;
      debug_image.encoding = sensor_msgs::image_encodings::RGB8;
      debug_image.image = rgb_flipped;

      image_cluster_pub_.publish(debug_image.toImageMsg());
    }
  }
}

void DynamicObstacleDetection::updateLoop(const ros::TimerEvent &time) {
  if (dvs_handler_.getSensorSize().width <= 0)
    return;
  if (dvs_handler_.needNewFrame()) {
    ros_time_ = ros::Time::now();
    bool computed = imu_warp_.computeMeanAngularVelocity();
    if (!computed)
    {
      return;
    }
    nmti_.lock();
    nmti_.drawNormalizedMeanTimestampImage();
    nmti_.meanFilterImage();
    nmti_.thresholdMeanTimestampImage();
    nmti_.filterImage();
    nmti_.removeBelowThreshold();
    nmti_.unlock();
    image_process_lock_.unlock();
  }
}

void DynamicObstacleDetection::initializeImage2World(
    Eigen::Matrix3d camera_mat, double focal_length, double width_real,
    double depth_uncertanty, double depth_object, Eigen::Matrix4d T_BC,
    double depth_limit) {
  image2world_ = Image2World(camera_mat, focal_length, width_real,
                             depth_uncertanty, depth_object, T_BC, depth_limit);
  image2world_initialized_ = true;
}

bool DynamicObstacleDetection::project2base(Eigen::Vector3d &world_point,
                    const double &width,
                    const Eigen::Vector2d &image_point) {
  if (width < 1e-10)
    return false;
  if (image2world_.project2base(world_point, width, image_point))
    return true;
}


void DynamicObstacleDetection::loadParameters() {
  update_frequency_ = param<int>(pnh_, "update_frequency", 100);
  
  // OpticalFlow
  of_window_width_ = (size_t)param<int>(pnh_, "optical_flow/window_width", 9);
  if (of_window_width_ % 2 == 0)
    LOG(ERROR) << "optical_flow_window_width is not an odd number: "
               << of_window_width_;
  of_inc_std_ = (float)param<double>(pnh_, "optical_flow/inc_std", 0.0);
  of_median_filter_width_ =
      (size_t)param<int>(pnh_, "optical_flow/median_filter_width", 0);
  of_mean_input_filter_width_ =
      (size_t)param<int>(pnh_, "optical_flow/mean_input_filter_width", 0);
  of_mean_output_filter_width_ =
      (size_t)param<int>(pnh_, "optical_flow/mean_output_filter_width", 0);

  // DBSCAN
  dbscan_eps_ = (float)param<double>(pnh_, "dbscan/eps", 0.5);
  dbscan_features_weights_.resize(3);
  dbscan_features_weights_[0] =
      (float)param<double>(pnh_, "dbscan/weights/pos", 1.0);
  dbscan_features_weights_[1] =
      (float)param<double>(pnh_, "dbscan/weights/vel", 1.0);
  dbscan_features_weights_[2] =
      (float)param<double>(pnh_, "dbscan/weights/timestamp", 1.0);
  dbscan_use_rotated_rect_ = param<bool>(pnh_, "dbscan/use_rotated_rect", true);

  // Backprojection
  width_real_ = param<double>(pnh_, "backprojection/width_real", 0.0);
  depth_uncertanty_ =
      param<double>(pnh_, "backprojection/depth_uncertanty", 0.0);
  depth_object_ = param<double>(pnh_, "backprojection/depth_object", 0.0);
  tolerance_ = param<double>(pnh_, "backprojection/tolerance", 0.0);

  depth_limit_ = param<double>(pnh_, "backprojection/depth_limit", 0.0);
  use_min_or_max_side_ = param<int>(pnh_, "backprojection/use_min_or_max_side", 1.0);

  std::vector<double> tmp;
  
  // Camera Offset
  std::vector<double> camera_offset;
  std::cout << typeid(pnh_).name() << '\n';
  if (pnh_.getParam("camera/offset", camera_offset)) {
    camera_offset_ = Eigen::Vector3d(camera_offset.data());
    LOG(INFO) << "[" << pnh_.getNamespace()
              << "] camare offset of size: " << camera_offset_.size()
              << " loaded: " << camera_offset_.transpose();
  } else {
    LOG(ERROR) << "[" << pnh_.getNamespace() << "] Could not load parameter "
               << pnh_.getNamespace() << "/camera_offset";
  }
  std::vector<double> camera_rot;
  if (pnh_.getParam("camera/rotation", camera_rot)) {
    if (camera_rot.size() == 3) {
      Eigen::Vector3d camera_rotation = Eigen::Vector3d(camera_rot.data());
      camera_rotation_ =
          quadrotor_common::eulerAnglesZYXToRotationMatrix(camera_rotation);
    } else if (camera_rot.size() == 4) {
      Eigen::Quaterniond camera_rotation;
      camera_rotation.w() = camera_rot[0];
      camera_rotation.x() = camera_rot[1];
      camera_rotation.y() = camera_rot[2];
      camera_rotation.z() = camera_rot[3];
      camera_rotation_ =
          quadrotor_common::quaternionToRotationMatrix(camera_rotation);
    } else {
      LOG(ERROR) << "[" << pnh_.getNamespace() << "] Wrong size "
                 << pnh_.getNamespace() << "/camera_offset";
    }
    LOG(INFO) << "[" << pnh_.getNamespace()
              << "] camare rotation of size: " << camera_rot.size()
              << " loaded: " << camera_rotation_;
  } else {
    LOG(ERROR) << "[" << pnh_.getNamespace() << "] Could not load parameter "
               << pnh_.getNamespace() << "/camera_offset";
  }
  std::vector<double> camera_mat;
  if (pnh_.getParam("camera/matrix", camera_mat)) {
    if (camera_mat.size() == 9) {
      camera_mat_ = Eigen::Matrix3d(camera_mat.data()).transpose();
    } else {
      LOG(ERROR) << "[" << pnh_.getNamespace() << "] Wrong size "
                 << pnh_.getNamespace() << "/camera_mat";
    }
    LOG(INFO) << "[" << pnh_.getNamespace()
              << "] camera matrix of size: " << camera_mat.size()
              << "PNH CAM loaded: " << camera_mat_;
  } else {
    LOG(ERROR) << "[" << pnh_.getNamespace() << "] Could not load parameter "
               << pnh_.getNamespace() << "/camera/matrix";
  }


}
} // namespace rpg_dynamic_obstacle_detection

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dynamic_obstacle_detection");

  rpg_dynamic_obstacle_detection::DynamicObstacleDetection
      dynamic_obstacle_detection;

  ros::spin();

  return 0;
}
