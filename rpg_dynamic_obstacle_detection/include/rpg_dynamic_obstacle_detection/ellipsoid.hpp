#pragma once

#include <Eigen/Eigen>
#include <glog/logging.h>

#include "rpg_dynamic_obstacle_detection/common.hpp"
#include "rpg_dynamic_obstacle_detection/sort_vectors.hpp"

namespace rpg_dynamic_obstacle_detection {
class Ellipsoid {
public:
  Ellipsoid(size_t dimensions);
  ~Ellipsoid(){};

  void minVolEllipsoid(const Eigen::MatrixXd &points, const double tolerance);
  void minVolEllipsoid(const std::vector<Eigen::VectorXd> &points,
                       const double tolerance);

  double distancePoint(const Eigen::Vector3d &point, Eigen::Vector3d &min_vec,
                       int max_iterations = 100);

  const Eigen::VectorXd &getCenter() const { return center_; }
  const Eigen::VectorXd &getEigenValues() const { return eigen_values_; }
  const Eigen::MatrixXd &getEigenVectors() const { return eigen_vectors_; }
  const Eigen::MatrixXd &getEllipsoidMatrix() const {
    return ellipsoid_matrix_;
  }
  const Eigen::VectorXd &getSemiAxes() const { return semi_axes_; }
  void changeCenter(const Eigen::Vector3d &center) {
    center_ = center;
  }

private:
  inline Eigen::Vector3d transform2FoREllipse(const Eigen::Vector3d &vec);
  inline Eigen::Vector3d rotate2FoRWorld(const Eigen::Vector3d &vec);
  double DistancePointEllipsoid(const Eigen::Vector3d &e,
                                const Eigen::Vector3d &y, Eigen::Vector3d &x,
                                const int &max_iterations);
  double DistancePointEllipse(const Eigen::Vector2d &e,
                              const Eigen::Vector2d &y, Eigen::Vector2d &x,
                              const int &max_iterations);
  double getRoot(double r0, double r1, double z0, double z1, double z2,
                 double g, int max_iterations);
  double getRoot(double r0, double z0, double z1, double g, int max_iterations);

  inline bool solveEigenvalueProblem();
  inline void arrangeAfterSemiAxes();

  size_t dimensions_;
  Eigen::VectorXd center_;
  Eigen::MatrixXd ellipsoid_matrix_;
  Eigen::VectorXd eigen_values_;
  Eigen::MatrixXd eigen_vectors_;
  Eigen::VectorXd semi_axes_;
};
} // namespace rpg_dynamic_obstacle_detection
