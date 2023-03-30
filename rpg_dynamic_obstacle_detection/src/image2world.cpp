#include "rpg_dynamic_obstacle_detection/image2world.hpp"

namespace rpg_dynamic_obstacle_detection {
Image2World::Image2World() {}

Image2World::Image2World(Eigen::Matrix3d camera_mat, double focal_length,
                         double width_real, double depth_uncertanty,
                         double depth_object, Eigen::Matrix4d T_BC,
                         double depth_limit)
    : camera_mat_(camera_mat), focal_length_(focal_length),
      width_real_(width_real), depth_uncertanty_(depth_uncertanty),
      depth_object_(depth_object), T_BC_(T_BC), depth_limit_(depth_limit) {
  camera_mat_inv_ = camera_mat_.inverse();
}

bool Image2World::project2world(
    std::vector<Eigen::Vector3d> &world_points, const double &width,
    const std::vector<Eigen::Vector2d> &image_points,
    const Eigen::Vector2d &center, const Eigen::Matrix3d &R_WB,
    const Eigen::Vector3d &t_WB) {
  double z_estimate = estimateZ(width);
  if (z_estimate > depth_limit_)
    return false;

  Eigen::Matrix4d T_WB = transformationMat(t_WB, R_WB);

  std::vector<Eigen::Vector3d> image_frame_points;
  image_frame_points.reserve(image_points.size() + 2);
  world_points.reserve(image_points.size() + 2);

  for (const Eigen::Vector2d &ip : image_points) {
    image_frame_points.push_back(projection(ip, z_estimate));
  }
  image_frame_points.push_back(
      projection(center, z_estimate - depth_object_ - depth_uncertanty_));
  image_frame_points.push_back(
      projection(center, z_estimate + depth_object_ + depth_uncertanty_));

  for (Eigen::Vector3d &ifp : image_frame_points) {
    Eigen::Vector4d tmp;
    Eigen::Vector4d ifp_h;
    ifp_h << ifp, 1.0;
    tmp = T_WB * T_BC_ * ifp_h;
    world_points.push_back(tmp.block<3, 1>(0, 0));
  }

  return true;
}

bool Image2World::project2base(
    Eigen::Vector3d &world_point, const double &width,
    const Eigen::Vector2d &image_point) {
  double z_estimate = estimateZ(width);
  if (z_estimate > depth_limit_)
    return false;

  Eigen::Vector3d image_frame_point;
  image_frame_point = projection(image_point, z_estimate);
  Eigen::Vector4d tmp;
  Eigen::Vector4d ifp_h;
  ifp_h << image_frame_point, 1.0;
  tmp = T_BC_ * ifp_h;
  world_point = tmp.block<3, 1>(0, 0);
  return true;
}

Eigen::Vector3d Image2World::projection(const Eigen::Vector2d &ip,
                                        const double &z) {
  Eigen::Vector3d ip_h;
  ip_h << ip, 1.0;
  double lamnda = z / (camera_mat_inv_.row(2) * ip_h);
  double x = lamnda * camera_mat_inv_.row(0) * ip_h;
  double y = lamnda * camera_mat_inv_.row(1) * ip_h;
  return Eigen::Vector3d(x, y, z);
}

double Image2World::estimateZ(const double &width_measured) {
  return focal_length_ * width_real_ / width_measured;
}

Eigen::Matrix4d Image2World::transformationMat(const Eigen::Vector3d &pos,
                                               const Eigen::Matrix3d &rot) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
  T.block<3, 3>(0, 0) = rot;
  T.block<3, 1>(0, 3) = pos;
  T(3, 3) = 1.0;
  return T;
}

Eigen::Matrix4d Image2World::transformationMatInv(const Eigen::Vector3d &pos,
                                                  const Eigen::Matrix3d &rot) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
  T.block<3, 3>(0, 0) = rot.transpose();
  T.block<3, 1>(0, 3) = -rot.transpose() * pos;
  T(3, 3) = 1.0;
  return T;
}
} // namespace rpg_dynamic_obstacle_detection
