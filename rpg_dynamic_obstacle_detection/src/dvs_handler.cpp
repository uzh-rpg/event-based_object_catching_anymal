#include "rpg_dynamic_obstacle_detection/dvs_handler.hpp"
#include "opencv2/calib3d/calib3d.hpp"

namespace rpg_dynamic_obstacle_detection {

DvsHandler::DvsHandler(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  loadParameters();
  cameraCalibration();
  event_sub_ = nh_.subscribe("events", 0, &DvsHandler::eventCallback, this);
  // event_imu_sub_ =
  //     nh_.subscribe("event_imu", 0, &DvsHandler::eventCallbackMod, this);
}

DvsHandler::~DvsHandler() {}

void DvsHandler::cameraCalibration() {
  camera_info_manager::CameraInfoManager cam_info(nh_, "DVXPLORER", url_);
  std::cout << "THE CAMERA URL IS " << url_ << std::endl;
  std::cout << "OPENCV VERSION: " << cv::getBuildInformation().c_str() << std::endl;
  image_geometry::PinholeCameraModel c;
  c.fromCameraInfo(cam_info.getCameraInfo());

  width_ = c.fullResolution().width;
  height_ = c.fullResolution().height;
  sensor_size_ = cv::Size(width_, height_);

  // Precompute rectification table
  for (size_t y = 0; y != height_; ++y) {
    for (size_t x = 0; x != width_; ++x) {
      points_.push_back(cv::Point2f(x, y));
    }
  }

  const std::string distortion_type = c.cameraInfo().distortion_model;
  LOG(INFO) << "Distortion type: " << distortion_type;
  // ADDED FOR DEBUG
  cv::Mat M = c.distortionCoeffs();
  LOG(INFO) << "Distortion coeffs: " << M;
  if (distortion_type == "plumb_bob") {
    std::cout << "USING DISTORTIONS" << std::endl;
    LOG(INFO) << "Using Plumb Bob Distortion";
    K_ref_ = cv::getOptimalNewCameraMatrix(
        c.fullIntrinsicMatrix(), c.distortionCoeffs(), sensor_size_, 0.);
    cv::undistortPoints(points_, rectified_points_, c.fullIntrinsicMatrix(),
                        c.distortionCoeffs(), cv::Matx33f::eye());
  } else if (distortion_type == "equidistant") {
    K_ref_ = cv::getOptimalNewCameraMatrix(
        c.fullIntrinsicMatrix(), c.distortionCoeffs(), sensor_size_, 0.);
    cv::undistortPoints(points_, rectified_points_, c.fullIntrinsicMatrix(),
                        c.distortionCoeffs(), cv::Matx33f::eye());
    /*cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        c.fullIntrinsicMatrix(), c.distortionCoeffs(), sensor_size_,
        cv::Matx33f::eye(), K_ref_);
    cv::fisheye::undistortPoints(points_, rectified_points_,
                                 c.fullIntrinsicMatrix(), c.distortionCoeffs(),
                                 cv::Matx33f::eye(), K_ref_);*/
  } else {
    CHECK(false) << "Distortion model: " << distortion_type
                 << " is not supported.";
  }

  LOG(INFO) << "Cam matrix "<< K_ref_;
  K_refx_ = cv::Matx33f(K_ref_);
  fx_ = K_refx_(0, 0);
  fy_ = K_refx_(1, 1);
  cx_ = K_refx_(0, 2);
  cy_ = K_refx_(1, 2);

  sensor_size_ = cv::Size(0, 0); // Why is the size set to 0,0 ??
}

void DvsHandler::init(int width, int height) {
  width_ = width;
  height_ = height;
  sensor_size_ = cv::Size(width_, height_);
}

void DvsHandler::clearEventQueue() {
  if (events_.size() > event_history_size_) {
    size_t remove_events = events_.size() - event_history_size_;

    events_.erase(events_.begin(), events_.begin() + remove_events);
    last_event_previous_frame_ -= remove_events;
  }
}

void DvsHandler::eventCallback(
    const dvs_msgs::EventArray::ConstPtr &event_array) {
  if (sensor_size_.width <= 0) {
    init(event_array->width, event_array->height);
  }

  clearEventQueue();
  for (const dvs_msgs::Event &e : event_array->events) {
    events_.push_back(e);
  }
}

// void DvsHandler::eventCallbackMod(
//     const ros_dvs_msgs::EventImuArray::ConstPtr &msg) {
//   if (sensor_size_.width <= 0) {
//     init(msg->event_array.width, msg->event_array.height);
//   }
//   clearEventQueue();
//   for (const ros_dvs_msgs::Event &e : msg->event_array.events) {
//     dvs_msgs::Event event;
//     event.x = e.x;
//     event.y = e.y;
//     event.ts = e.ts;
//     event.polarity = e.polarity;
//     events_.push_back(event);
//   }
// }

bool DvsHandler::needNewFrameMilliseconds() {
  const size_t last_event = events_.size() - 1;
  const ros::Time stamp_last_event = events_[last_event].ts;
  if ((stamp_last_event - events_[0].ts).toSec() < frame_size_) {
    LOG(WARNING) << "Time scope of events in the queue ("
                 << (stamp_last_event - events_[0].ts).toSec()
                 << ") not enough to create a frame of desired duration: "
                 << frame_size_ << " s";
    return false;
  }

  if ((stamp_last_event - stamp_last_event_previous_frame_).toSec() <
      min_step_size_) {
    VLOG(10) << "Ignoring request for motion correction. Last event: "
             << stamp_last_event << ", last_event_previous_frame: "
             << stamp_last_event_previous_frame_;
    return false;
  }

  // rewind the event queue until we found the desired stamps frame_begin,
  // frame_end
  frame_end_ = last_event;
  VLOG(30) << "frame_end/ event: " << last_event
           << " stamp: " << events_[last_event].ts;
  for (size_t ev_idx = last_event; ev_idx >= 0; ev_idx--) {
    double dt = (stamp_last_event - events_[ev_idx].ts).toSec();
    if (dt >= frame_size_) {
      frame_begin_ = ev_idx;
      VLOG(30) << "frame_begin / event: " << ev_idx
               << " stamp: " << events_[ev_idx].ts;
      break;
    }
  }

  stamp_last_event_previous_frame_ = stamp_last_event;

  return true;
}

bool DvsHandler::needNewFrame() {
  bool check = needNewFrameMilliseconds();
  if (check) {
    if (frame_end_ - frame_begin_ < min_num_events_per_frame_) {
      VLOG(2) << "Frame does not have enough events ( "
              << (frame_end_ - frame_begin_) << " / "
              << min_num_events_per_frame_ << " ). Ignoring.";
      return false;
    }
    return true;
  }
  return false;
}

void DvsHandler::loadParameters() {
  // Load camera calibration
  const std::string calib_file =
      rpg_dynamic_obstacle_detection::param<std::string>(pnh_, "calib_file",
                                                         "");
  url_ = std::string("file://") + calib_file;
  LOG(INFO) << "Calib File: " << url_;

  min_num_events_per_frame_ = rpg_dynamic_obstacle_detection::param<int>(
      pnh_, "dvs_handler/min_num_events_per_frame", 1000);

  const size_t frame_size_ms = rpg_dynamic_obstacle_detection::param<int>(
      pnh_, "dvs_handler/frame_size", 10);
  const size_t min_step_size_ms = rpg_dynamic_obstacle_detection::param<int>(
      pnh_, "dvs_handler/min_step_size", 1);
  frame_size_ = static_cast<double>(frame_size_ms / 1000.0);
  min_step_size_ = static_cast<double>(min_step_size_ms / 1000.0);

  event_history_size_ = rpg_dynamic_obstacle_detection::param<int>(
      pnh_, "dvs_handler/event_history_size", 5000000);
}
} // namespace rpg_dynamic_obstacle_detection
