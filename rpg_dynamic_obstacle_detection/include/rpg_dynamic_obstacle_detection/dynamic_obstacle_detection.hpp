#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <glog/logging.h>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <rpg_dynamic_obstacle_detection/DynamicObstacleDetectionConfig.h>
#include "quadrotor_common/math_common.h"

#include "rpg_dynamic_obstacle_detection/average_timer.hpp"
#include "rpg_dynamic_obstacle_detection/clustering.hpp"
#include "rpg_dynamic_obstacle_detection/common.hpp"
#include "rpg_dynamic_obstacle_detection/dvs_handler.hpp"
#include "rpg_dynamic_obstacle_detection/imu_warp.hpp"
#include "rpg_dynamic_obstacle_detection/normalized_mean_timestamp_image.hpp"
#include "rpg_dynamic_obstacle_detection/param.hpp"

#include "rpg_dynamic_obstacle_detection_msgs/OpticalFlowImages.h"
#include "rpg_dynamic_obstacle_detection_msgs/BoundingBox2DStamped.h"
#include "rpg_dynamic_obstacle_detection/image2world.hpp"

#include "rpg_dynamic_obstacle_detection/timer.h"
#include <fstream>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif /* ifndef M_PI */

namespace rpg_dynamic_obstacle_detection {

class DynamicObstacleDetection {
public:
  DynamicObstacleDetection(ros::NodeHandle nh, ros::NodeHandle pnh);
  DynamicObstacleDetection()
      : DynamicObstacleDetection(ros::NodeHandle(), ros::NodeHandle("~")) {}
  ~DynamicObstacleDetection() {file_handle_.close();}

private:
  void loadParameters();

  std::ofstream file_handle_;


  AverageTimer timer_1_;
  AverageTimer timer_2_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // For debug testing
  ros::Publisher image_pub_;
  ros::Publisher optical_flow_pub_;
  ros::Publisher image_cluster_pub_;
  ros::Publisher average_timestamp_pub_;
  cv::Mat debug_image_;

  ros::Publisher obstacles_pose_pub_;
  ros::Publisher obstacle_detected_bool_pub_;

  ros::Publisher marker_pub_;
  double marker_lifetime_;
  double marker_decay_factor_;
  size_t visualization_update_frequency_;
  ros::Timer visualization_update_timer_;
  unsigned long marker_it_ = 0;
  ros::Publisher imu_warp_pub_;
  ros::Publisher cluster_center_pub_;

  size_t update_frequency_;
  ros::Timer update_timer_;
  void updateLoop(const ros::TimerEvent &time);

  std::thread image_process_;
  void imageProcessing();
  std::mutex image_process_lock_;

  void reconfigureCallback(DynamicObstacleDetectionConfig &config,
                           uint32_t level);
  dynamic_reconfigure::Server<DynamicObstacleDetectionConfig> reconfigure_srv_;
  bool reconfigure_was_called_before_ = false;

  DvsHandler dvs_handler_;
  ImuWarp imu_warp_;
  NormalizedMeanTimestampImage nmti_;

  // OpticalFlow
  size_t of_window_width_;
  double of_inc_std_;
  size_t of_median_filter_width_;
  size_t of_mean_input_filter_width_;
  size_t of_mean_output_filter_width_;

  Clustering dbscan_;
  float dbscan_eps_;
  std::vector<float> dbscan_features_weights_;
  bool dbscan_use_rotated_rect_;

  Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
  double desired_velocity_ = 0.0;

  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
  size_t update_frequency_velocity_;
  ros::Timer potential_field_update_timer_;
  void velocityUpdateLoop(const ros::TimerEvent &update_time);

  Eigen::Vector3d camera_offset_;
  Eigen::Matrix3d camera_rotation_;

  // Backprojection
  void initializeImage2World(Eigen::Matrix3d camera_mat, double focal_length,
                             double width_real, double depth_uncertanty,
                             double depth_object, Eigen::Matrix4d T_BC,
                             double depth_limit);
  
  bool project2base(Eigen::Vector3d &world_point,
                    const double &width,
                    const Eigen::Vector2d &image_point);
  
  bool image2world_initialized_;
  bool potential_field_initialized_;
  bool ellipsoid_initialized_;
  Image2World image2world_;

  Eigen::Matrix3d camera_mat_;
  double focal_length_;
  double width_real_;
  double depth_uncertanty_;
  double depth_object_;
  double depth_limit_;
  // Ellipsoid
  double tolerance_;

  double rectangle_ratio_limit_;
  int use_min_or_max_side_;
  ros::Time ros_time_;
  bool use_optimized_camera_matrix_;

  float position_x = 0.0;
  float position_y = 0.0;
  float position_z = -1.0;
};
} // namespace rpg_dynamic_obstacle_detection
