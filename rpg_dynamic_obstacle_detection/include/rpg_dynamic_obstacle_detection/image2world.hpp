#pragma once

#include <Eigen/Eigen>
#include <iostream>

namespace rpg_dynamic_obstacle_detection {
class Image2World {
public:
  Image2World();
  Image2World(Eigen::Matrix3d camera_mat, double focal_length,
              double width_real, double depth_uncertanty, double depth_object,
              Eigen::Matrix4d T_BC, double depth_limit);
  ~Image2World() {}

  bool project2world(std::vector<Eigen::Vector3d> &world_points,
                     const double &width,
                     const std::vector<Eigen::Vector2d> &image_points,
                     const Eigen::Vector2d &center, const Eigen::Matrix3d &R_WB,
                     const Eigen::Vector3d &t_WB);
  bool project2base(Eigen::Vector3d &world_point,
                    const double &width,
                    const Eigen::Vector2d &image_point);

private:
  inline double estimateZ(const double &width_measured);
  inline Eigen::Vector3d projection(const Eigen::Vector2d &ip, const double &z);

  inline Eigen::Matrix4d transformationMat(const Eigen::Vector3d &pos,
                                           const Eigen::Matrix3d &rot);
  inline Eigen::Matrix4d transformationMatInv(const Eigen::Vector3d &pos,
                                              const Eigen::Matrix3d &rot);

  Eigen::Matrix3d camera_mat_;
  Eigen::Matrix3d camera_mat_inv_;
  Eigen::Matrix4d T_BC_;
  double focal_length_;
  double width_real_;
  double depth_uncertanty_;
  double depth_object_;

  double depth_limit_;
};
} // namespace rpg_dynamic_obstacle_detection
