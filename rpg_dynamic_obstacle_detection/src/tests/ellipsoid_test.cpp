#include <Eigen/Eigen>

#include "rpg_dynamic_obstacle_detection/ellipsoid.hpp"

using namespace rpg_dynamic_obstacle_detection;
int main(int argc, char *argv[]) {
  std::vector<Eigen::VectorXd> points2d(4);
  std::vector<Eigen::VectorXd> points3d(6);

  points2d[0] = Eigen::Vector2d(0, 2);
  points2d[1] = Eigen::Vector2d(0, -2);
  points2d[2] = Eigen::Vector2d(1, 0);
  points2d[3] = Eigen::Vector2d(-1, 0);

  points3d[0] = Eigen::Vector3d(0, 3, 0);
  points3d[1] = Eigen::Vector3d(0, -3, 0);
  points3d[2] = Eigen::Vector3d(2, 0, 0);
  points3d[3] = Eigen::Vector3d(-2, 0, 0);
  points3d[4] = Eigen::Vector3d(0, 0, 1);
  points3d[5] = Eigen::Vector3d(0, 0, -1);

  Ellipsoid ellipsoid2d(2);
  Ellipsoid ellipsoid3d(3);

  ellipsoid2d.minVolEllipsoid(points2d, 1e-3);

  std::cout << "2D:\n"
            << "Center:\n"
            << ellipsoid2d.getCenter() << "\nSemi Axes:\n"
            << ellipsoid2d.getSemiAxes() << "\nEigenValues:\n"
            << ellipsoid2d.getEigenValues() << "\nEigenVectors:\n"
            << ellipsoid2d.getEigenVectors() << std::endl;

  ellipsoid3d.minVolEllipsoid(points3d, 1e-6);

  std::cout << "\n\n3D:\n"
            << "Center:\n"
            << ellipsoid3d.getCenter() << "\nSemi Axes:\n"
            << ellipsoid3d.getSemiAxes() << "\nEigenValues:\n"
            << ellipsoid3d.getEigenValues() << "\nEigenVectors:\n"
            << ellipsoid3d.getEigenVectors() << std::endl;

  Eigen::Vector3d tmp;
  Eigen::Vector3d point;
  point << 0.75, 0.5, 0;
  double dist = ellipsoid3d.distancePoint(point, tmp, 500);

  std::cout << "distance:\n" << dist << "\nmin vector:\n" << tmp << std::endl;

  return 0;
}
