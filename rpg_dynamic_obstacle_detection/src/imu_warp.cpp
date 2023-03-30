#include <rpg_dynamic_obstacle_detection/imu_warp.hpp>

namespace rpg_dynamic_obstacle_detection {
ImuWarp::ImuWarp(ros::NodeHandle nh, ros::NodeHandle pnh,
                 const EventQueue &events, const size_t &frame_begin,
                 const size_t &frame_end)
    : nh_(nh), pnh_(pnh), events_(events), frame_begin_(frame_begin),
      frame_end_(frame_end) {
  imu_sub_ = nh_.subscribe("imu", 0, &ImuWarp::imuCallback, this);
  //imu_sub_ = nh_.subscribe("event_imu", 0, &ImuWarp::imuCallbackMod, this);
}

bool ImuWarp::computeMeanAngularVelocity() {
  static unsigned long it = 0;
  ++it;
  ze::ImuStamps imu_timestamps;
  ze::ImuAccGyrContainer imu_measurements;

  int64_t oldest_stamp, newest_stamp;
  bool res;
  imu_mutex_.lock();
  std::tie(oldest_stamp, newest_stamp, res) =
      imu_buffer_.getOldestAndNewestStamp();

  int64_t frame_begin_ts = (int64_t)events_[frame_begin_].ts.toNSec();

  if (frame_begin_ts < oldest_stamp) {
    LOG(WARNING) << "Oldest IMU stamp: " << oldest_stamp
                 << " is newer than the start of the event frame: "
                 << frame_begin_ts << " iteration: " << it;
    warp_ = cv::Mat::zeros(3, 1, CV_64F);
    imu_mutex_.unlock();
    return false;
  }

  if (frame_begin_ts >= newest_stamp) {
    LOG(WARNING) << "Newest IMU stamp is older than the start of the event "
                    "frame. Time Stamp Difference: "
                 << (double)(frame_begin_ts - newest_stamp) * 1.0e-6
                 << "[ms]. iteration: " << it;
    warp_ = cv::Mat::zeros(3, 1, CV_64F);
    imu_mutex_.unlock();
    return false;
  }

  const int64_t end_time =
      std::min(newest_stamp, (int64_t)events_[frame_end_].ts.toNSec());
  std::tie(imu_timestamps, imu_measurements) =
      imu_buffer_.getBetweenValuesInterpolated(frame_begin_ts, end_time);
  imu_mutex_.unlock();

  VLOG(10) << "Num IMU measurements: " << imu_timestamps.rows();

  ze::Vector3 mean_w_body(0.0, 0.0, 0.0);
  for (size_t i = 0; i < imu_timestamps.rows() - 1; ++i) {
    ze::real_t dt =
        (ze::real_t)(imu_timestamps(i + 1) - imu_timestamps(i)) / 1000000000.0;
    VLOG(50) << imu_measurements.col(i).tail<3>().transpose();
    VLOG(50) << dt;
    mean_w_body += (dt * imu_measurements.col(i).tail<3>());
  }
  mean_w_body /= (ze::real_t)(imu_timestamps(imu_timestamps.rows() - 1) -
                              imu_timestamps(0)) /
                 1000000000.0;
  cv::Mat w_body;
  w_body.create(mean_w_body.rows(), mean_w_body.cols(), CV_64F);
  Eigen::Map<Eigen::Matrix<double, 3, 1>>(
      reinterpret_cast<double *>(w_body.data), mean_w_body.rows(),
      mean_w_body.cols()) = mean_w_body.cast<double>();

  // Transform from IMU frame to camera frame
  //!!TODO!!: this works only if the IMU and camera frames are aligned
  // done? hopefully correct
  //!!!! -w_body is right, switched for testing !!!!!
  warp_ = -w_body;

  return true;
}

void ImuWarp::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  ze::ImuAccGyr acc_gyr;
  int64_t stamp = (int64_t)msg->header.stamp.toNSec();
  acc_gyr(0) = msg->linear_acceleration.x;
  acc_gyr(1) = msg->linear_acceleration.y;
  acc_gyr(2) = msg->linear_acceleration.z;
  acc_gyr(3) = msg->angular_velocity.x;
  acc_gyr(4) = msg->angular_velocity.y;
  acc_gyr(5) = msg->angular_velocity.z;

  imu_mutex_.lock();
  imu_buffer_.insert(stamp, acc_gyr);
  imu_mutex_.unlock();
}

// void ImuWarp::imuCallbackMod(const ros_dvs_msgs::EventImuArray::ConstPtr &msg) {
//   ze::ImuAccGyr acc_gyr;
//   for (const auto &m : msg->imu_array) {
//     int64_t stamp = (int64_t)m.header.stamp.toNSec();
//     acc_gyr(0) = m.linear_acceleration.x;
//     acc_gyr(1) = m.linear_acceleration.y;
//     acc_gyr(2) = m.linear_acceleration.z;
//     acc_gyr(3) = m.angular_velocity.x;
//     acc_gyr(4) = m.angular_velocity.y;
//     acc_gyr(5) = m.angular_velocity.z;

//     imu_mutex_.lock();
//     imu_buffer_.insert(stamp, acc_gyr);
//     imu_mutex_.unlock();
//   }
// }

void ImuWarp::loadParameters() {}
} // namespace rpg_dynamic_obstacle_detection
