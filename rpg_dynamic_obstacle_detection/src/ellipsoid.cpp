#include "rpg_dynamic_obstacle_detection/ellipsoid.hpp"

namespace rpg_dynamic_obstacle_detection {
Ellipsoid::Ellipsoid(size_t dimensions = 3) : dimensions_(dimensions) {}

double Ellipsoid::distancePoint(const Eigen::Vector3d &point,
                                Eigen::Vector3d &min_vec, int max_iterations) {
  Eigen::Vector3d point_E = transform2FoREllipse(point);
  Eigen::Vector3d octant_dir = point_E.array().sign();
  point_E = point_E.cwiseProduct(octant_dir);
  double distance =
      DistancePointEllipsoid(semi_axes_, point_E, min_vec, max_iterations);
  min_vec = min_vec.cwiseProduct(octant_dir);
  min_vec = rotate2FoRWorld(min_vec);
  return distance;
}

Eigen::Vector3d Ellipsoid::transform2FoREllipse(const Eigen::Vector3d &vec) {
  Eigen::Vector3d out = vec - center_;
  out = eigen_vectors_.transpose() * out;
  return out;
}

Eigen::Vector3d Ellipsoid::rotate2FoRWorld(const Eigen::Vector3d &vec) {
  return eigen_vectors_ * vec;
}

double Ellipsoid::getRoot(double r0, double z0, double z1, double g,
                          int max_iterations) {
  /* Distance from a Point to an Ellipse, an Ellipsoid, or a
   * Hyperellipsoid
   * David Eberly, Geometric Tools, Redmond WA 98052
   */
  double n0 = r0 * z0;
  double s0 = z1 - 1.0;
  double s1 = (g < 0.0 ? 0.0 : Eigen::Vector2d(n0, z1).norm() - 1.0);
  double s = 0.0;
  for (int i = 0; i < max_iterations; ++i) {
    s = (s0 + s1) / 2.0;
    if (s == s0 || s == s1) {
      break;
    }
    double ratio0 = n0 / (s + r0);
    double ratio1 = z1 / (s + 1.0);
    g = square(ratio0) + square(ratio1) - 1.0;
    if (g > 0.0) {
      s0 = s;
    } else if (g < 0.0) {
      s1 = s;
    } else {
      break;
    }
  }
  return s;
}

double Ellipsoid::getRoot(double r0, double r1, double z0, double z1, double z2,
                          double g, int max_iterations) {
  /* Distance from a Point to an Ellipse, an Ellipsoid, or a
   * Hyperellipsoid
   * David Eberly, Geometric Tools, Redmond WA 98052
   */
  double n0 = r0 * z0;
  double n1 = r1 * z1;
  double s0 = z1 - 1.0;
  double s1 = (g < 0.0 ? 0.0 : Eigen::Vector3d(n0, n1, z2).norm() - 1.0);
  double s = 0.0;
  for (int i = 0; i < max_iterations; ++i) {
    s = (s0 + s1) / 2.0;
    if (s == s0 || s == s1) {
      break;
    }
    double ratio0 = n0 / (s + r0);
    double ratio1 = n1 / (s + r1);
    double ratio2 = z2 / (s + 1.0);
    g = square(ratio0) + square(ratio1) + square(ratio2) - 1.0;
    if (g > 0.0) {
      s0 = s;
    } else if (g < 0.0) {
      s1 = s;
    } else {
      break;
    }
  }
  return s;
}

double Ellipsoid::DistancePointEllipse(const Eigen::Vector2d &e,
                                       const Eigen::Vector2d &y,
                                       Eigen::Vector2d &x,
                                       const int &max_iterations) {
  /* Distance from a Point to an Ellipse, an Ellipsoid, or a
   * Hyperellipsoid
   * David Eberly, Geometric Tools, Redmond WA 98052
   */
  double distance;
  if (y(1) > 0.0) {
    if (y(0) > 0.0) {
      double z0 = y(0) / e(0);
      double z1 = y(1) / e(1);
      double g = square(z0) + square(z1) - 1.0;
      if (g != 0.0) {
        double r0 = square(e(0) / e(1));
        double sbar = getRoot(r0, z0, z1, g, max_iterations);
        x(0) = r0 * y(0) / (sbar + r0);
        x(1) = y(1) / (sbar + 1.0);
        distance = (x - y).norm();
      } else {
        x(0) = y(0);
        x(1) = y(1);
        distance = 0.0;
      }
    } else {
      // y(0) == 0
      x(0) = 0.0;
      x(1) = e(1);
      distance = std::fabs(y(1) - e(1));
    }
  } else {
    // y(1) == 0
    double numer0 = e(0) * y(0);
    double denom0 = square(e(0)) - square(e(1));
    if (numer0 < denom0) {
      double xde0 = numer0 / denom0;
      x(0) = e(0) * xde0;
      x(1) = e(1) * std::sqrt(1.0 - xde0 * xde0);
      if (y(1) != 0.0)
        LOG(WARNING) << "y(1) should be 0.0, but is: " << y(1);
      distance = (x - y).norm();
    } else {
      x(0) = e(0);
      x(1) = 0.0;
      distance = std::fabs(y(0) - e(0));
    }
  }
  return distance;
}

double Ellipsoid::DistancePointEllipsoid(const Eigen::Vector3d &e,
                                         const Eigen::Vector3d &y,
                                         Eigen::Vector3d &x,
                                         const int &max_iterations) {
  /* Distance from a Point to an Ellipse, an Ellipsoid, or a
   * Hyperellipsoid
   * David Eberly, Geometric Tools, Redmond WA 98052
   */
  if (std::pow(y.x() / e.x(), 2) + std::pow(y.y() / e.y(), 2) +
          std::pow(y.z() / e.z(), 2) <=
      1.0)
    return 0.0;

  double distance;
  if (y(2) > 0.0) {
    if (y(1) > 0.0) {
      if (y(0) > 0.0) {
        double z0 = y(0) / e(0);
        double z1 = y(1) / e(1);
        double z2 = y(2) / e(2);
        double g = square(z0) + square(z1) + square(z2) - 1.0;
        if (g != 0.0) {
          double r0 = square(e(0) / e(2));
          double r1 = square(e(1) / e(2));
          double sbar = getRoot(r0, r1, z0, z1, z2, g, max_iterations);
          x(0) = r0 * y(0) / (sbar + r0);
          x(1) = r1 * y(1) / (sbar + r1);
          x(2) = y(2) / (sbar + 1.0);
          distance = (x - y).norm();
        } else {
          x(0) = y(0);
          x(1) = y(1);
          x(2) = y(2);
          distance = 0.0;
        }
      } else {
        // y(0) == 0
        x(0) = 0.0;
        Eigen::Vector2d e_tmp(e(1), e(2));
        Eigen::Vector2d y_tmp(y(1), y(2));
        Eigen::Vector2d x_tmp(x(1), x(2));
        distance = DistancePointEllipse(e_tmp, y_tmp, x_tmp, max_iterations);
        x(1) = x_tmp(1);
        x(2) = x_tmp(2);
      }
    } else {
      // y(1) == 0
      if (y(0) > 0.0) {
        x(1) = 0.0;
        Eigen::Vector2d e_tmp(e(0), e(2));
        Eigen::Vector2d y_tmp(y(0), y(2));
        Eigen::Vector2d x_tmp(x(0), x(2));
        distance = DistancePointEllipse(e_tmp, y_tmp, x_tmp, max_iterations);
        x(0) = x_tmp(0);
        x(2) = x_tmp(2);
      } else {
        // y(0) == 0.0
        x(0) = 0.0;
        x(1) = 0.0;
        x(2) = e(2);
        distance = std::fabs(y(2) - e(2));
      }
    }
  } else {
    // y(2) == 0
    double denom0 = e(0) * e(0) - e(2) * e(2);
    double denom1 = e(1) * e(1) - e(2) * e(2);
    double numer0 = e(0) * y(0);
    double numer1 = e(1) * y(1);
    bool computed = false;
    if (numer0 < denom0 && numer1 < denom1) {
      double xde0 = numer0 / denom0;
      double xde1 = numer1 / denom1;
      double xde0sqr = xde0 * xde0;
      double xde1sqr = xde1 * xde1;
      double discr = 1.0 - xde0sqr - xde1sqr;
      if (discr > 0.0) {
        x(0) = e(0) * xde0;
        x(1) = e(1) * xde1;
        x(2) = e(2) * std::sqrt(discr);
        if (y(2) != 0.0)
          LOG(WARNING) << "y(1) should be 0.0, but is: " << y(1);
        distance = (x - y).norm();
        computed = true;
      }
    }
    if (!computed) {
      x(2) = 0.0;
      Eigen::Vector2d e_tmp(e(0), e(1));
      Eigen::Vector2d y_tmp(y(0), y(1));
      Eigen::Vector2d x_tmp(x(0), x(1));
      distance = DistancePointEllipse(e_tmp, y_tmp, x_tmp, max_iterations);
      x(0) = x_tmp(0);
      x(1) = x_tmp(1);
    }
  }
  return distance;
}

bool Ellipsoid::solveEigenvalueProblem() {
  Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver;
  eigen_solver.compute(ellipsoid_matrix_, true);
  eigen_values_ = eigen_solver.eigenvalues().real();
  eigen_vectors_ = eigen_solver.eigenvectors().real();

  if (eigen_values_.size() != dimensions_) {
    LOG(ERROR) << "ERRROOOORRR  dim should:" << dimensions_
               << " dim is: " << eigen_values_.size();
    std::cout << ellipsoid_matrix_ << std::endl;
    return false;
  }
  return true;
}

void Ellipsoid::arrangeAfterSemiAxes() {
  std::vector<double> semi_axes;
  std::vector<double> eigen_values;
  semi_axes.resize(semi_axes_.size());
  eigen_values.resize(eigen_values_.size());
  Eigen::VectorXd::Map(&semi_axes[0], semi_axes_.size()) = semi_axes_;
  Eigen::VectorXd::Map(&eigen_values[0], eigen_values_.size()) = eigen_values_;
  std::vector<Eigen::VectorXd> eigen_vectors;
  eigen_vectors.resize(dimensions_);
  for (int i = 0; i < dimensions_; ++i) {
    eigen_vectors[i] = eigen_vectors_.col(i);
  }
  sortVectorsDescending(semi_axes, semi_axes, eigen_values, eigen_vectors);
  for (int i = 0; i < dimensions_; ++i) {
    semi_axes_[i] = semi_axes[i];
    eigen_values_[i] = eigen_values[i];
    eigen_vectors_.col(i) = eigen_vectors[i];
  }
}

void Ellipsoid::minVolEllipsoid(const Eigen::MatrixXd &points,
                                const double tolerance) {
  /* Adaptation of:
   * MINIMUM VOLUME ENCLOSING ELLIPSOIDS
   * NIMA MOSHTAGH
   * NIMA@GRASP.UPENN
   * University of Pennsylvania
   * December 2005
   */
  if (points.rows() != dimensions_)
    LOG(ERROR) << "Point dimensions not equal to specified dimensions! :"
               << points.cols() << " | " << dimensions_;
  int d = dimensions_;
  int N = points.cols();

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(d + 1, N);
  for (int i = 0; i < d; ++i) {
    Q.row(i) = points.row(i);
  }
  Q.row(d) = Eigen::RowVectorXd::Ones(N);

  size_t count = 1;
  double err = 1.0;
  Eigen::VectorXd u = Eigen::VectorXd::Ones(N) / (double)N;

  while (err > tolerance) {
    // X = \sum_i ( u_i * q_i * q_i')  is a (d+1)x(d+1) matrix
    Eigen::MatrixXd X;
    X = Q * u.asDiagonal() * Q.transpose();
    // M the diagonal vector of an NxN matrix
    Eigen::VectorXd M;
    M = (Q.transpose() * X.inverse() * Q).diagonal();
    int j;
    double maximum = M.maxCoeff(&j);
    double step_size =
        (maximum - (double)d - 1.0) / (((double)d + 1.0) * (maximum - 1.0));
    Eigen::VectorXd new_u = (1.0 - step_size) * u;
    new_u(j) = new_u(j) + step_size;
    count += 1;
    err = (new_u - u).norm();
    u = new_u;
  }

  // Finds the ellipse equation in the 'center form':
  // (x-c)' * A * (x-c) = 1
  // It computes a dxd matrix 'A' and a d dimensional vector 'c' as the center
  // of the ellipse.
  // ellipsoid_matrix_ = A | center_ = c
  Eigen::MatrixXd U = u.asDiagonal();
  ellipsoid_matrix_ = (points * U * points.transpose() -
                       (points * u) * (points * u).transpose())
                          .inverse() /
                      (double)d;
  center_ = points * u;

  if (!solveEigenvalueProblem()) {
    std::cout << "points:\n" << points << std::endl;
    std::cout << "u:\n" << u << std::endl;
  }

  semi_axes_ = Eigen::VectorXd::Zero(dimensions_);
  for (int i = 0; i < dimensions_; ++i) {
    if (eigen_values_[i] != 0.0) {
      semi_axes_[i] = 1.0 / std::sqrt(eigen_values_[i]);
    }
  }
  // std::cout << semi_axes_ << std::endl;

  arrangeAfterSemiAxes();
}

void Ellipsoid::minVolEllipsoid(const std::vector<Eigen::VectorXd> &points,
                                const double tolerance) {
  Eigen::MatrixXd points_matrix(points.front().size(), points.size());
  for (int i = 0; i < points.size(); ++i) {
    points_matrix.col(i) = points[i];
  }
  minVolEllipsoid(points_matrix, tolerance);
}
} // namespace rpg_dynamic_obstacle_detection
