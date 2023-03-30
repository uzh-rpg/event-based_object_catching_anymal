#pragma once

#include <Eigen/Eigen>
#include <random>

namespace rpg_dynamic_obstacle_detection {
struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d pos;
  Eigen::Vector2d vel;
  double area;
  double angle;
  double width;
  double height;
  double mean_timestamp;

  State operator+(const State &s) {
    State state;
    state.pos(0) = this->pos(0) + s.pos(0);
    state.pos(1) = this->pos(1) + s.pos(1);
    state.vel(0) = this->vel(0) + s.vel(0);
    state.vel(1) = this->vel(1) + s.vel(1);
    state.area = this->area + s.area;
    state.width = this->width + s.width;
    state.height = this->height + s.height;
    state.angle = this->angle + s.angle;
    state.mean_timestamp = this->mean_timestamp + s.mean_timestamp;
    return state;
  }

  State operator-(const State &s) {
    State state;
    state.pos(0) = this->pos(0) - s.pos(0);
    state.pos(1) = this->pos(1) - s.pos(1);
    state.vel(0) = this->vel(0) - s.vel(0);
    state.vel(1) = this->vel(1) - s.vel(1);
    state.area = this->area - s.area;
    state.width = this->width - s.width;
    state.height = this->height - s.height;
    state.angle = this->angle - s.angle;
    state.mean_timestamp = this->mean_timestamp - s.mean_timestamp;
    return state;
  }

  State operator*(const State &s) {
    State state;
    state.pos(0) = this->pos(0) * s.pos(0);
    state.pos(1) = this->pos(1) * s.pos(1);
    state.vel(0) = this->vel(0) * s.vel(0);
    state.vel(1) = this->vel(1) * s.vel(1);
    state.area = this->area * s.area;
    state.width = this->width * s.width;
    state.height = this->height * s.height;
    state.angle = this->angle * s.angle;
    state.mean_timestamp = this->mean_timestamp * s.mean_timestamp;
    return state;
  }

  State operator/(const State &s) {
    State state;
    state.pos(0) = this->pos(0) / s.pos(0);
    state.pos(1) = this->pos(1) / s.pos(1);
    state.vel(0) = this->vel(0) / s.vel(0);
    state.vel(1) = this->vel(1) / s.vel(1);
    state.area = this->area / s.area;
    state.width = this->width / s.width;
    state.height = this->height / s.height;
    state.angle = this->angle / s.angle;
    state.mean_timestamp = this->mean_timestamp / s.mean_timestamp;
    return state;
  }

  State operator*(const double &d) {
    State state;
    state.pos(0) = this->pos(0) * d;
    state.pos(1) = this->pos(1) * d;
    state.vel(0) = this->vel(0) * d;
    state.vel(1) = this->vel(1) * d;
    state.area = this->area * d;
    state.width = this->width * d;
    state.height = this->height * d;
    state.angle = this->angle * d;
    state.mean_timestamp = this->mean_timestamp * d;
    return state;
  }

  State operator/(const double &d) {
    State state;
    state.pos(0) = this->pos(0) / d;
    state.pos(1) = this->pos(1) / d;
    state.vel(0) = this->vel(0) / d;
    state.vel(1) = this->vel(1) / d;
    state.area = this->area / d;
    state.width = this->width / d;
    state.height = this->height / d;
    state.angle = this->angle / d;
    state.mean_timestamp = this->mean_timestamp / d;
    return state;
  }
};

class StateNormalDistribution {
public:
  StateNormalDistribution() : gen_(rd_()) {}
  inline double pos_x() { return pos_x_(gen_); }
  inline double pos_y() { return pos_y_(gen_); }
  inline double vel_x() { return vel_x_(gen_); }
  inline double vel_y() { return vel_y_(gen_); }
  inline double area() { return area_(gen_); }
  inline double width() { return width_(gen_); }
  inline double height() { return height_(gen_); }
  inline double angle() { return angle_(gen_); }
  inline double mean_timestamp() { return mean_timestamp_(gen_); }
  void init(const State &mean, const State &std) {
    pos_x_ = std::normal_distribution<double>(mean.pos(0), std.pos(0));
    pos_y_ = std::normal_distribution<double>(mean.pos(1), std.pos(1));
    vel_x_ = std::normal_distribution<double>(mean.vel(0), std.vel(0));
    vel_y_ = std::normal_distribution<double>(mean.vel(1), std.vel(1));
    area_ = std::normal_distribution<double>(mean.area, std.area);
    width_ = std::normal_distribution<double>(mean.width, std.width);
    height_ = std::normal_distribution<double>(mean.height, std.height);
    angle_ = std::normal_distribution<double>(mean.angle, std.angle);
    mean_timestamp_ = std::normal_distribution<double>(mean.mean_timestamp,
                                                       std.mean_timestamp);
  }

private:
  std::random_device rd_;
  std::mt19937 gen_;
  std::normal_distribution<double> pos_x_;
  std::normal_distribution<double> pos_y_;
  std::normal_distribution<double> vel_x_;
  std::normal_distribution<double> vel_y_;
  std::normal_distribution<double> area_;
  std::normal_distribution<double> width_;
  std::normal_distribution<double> height_;
  std::normal_distribution<double> angle_;
  std::normal_distribution<double> mean_timestamp_;
};

} // namespace rpg_dynamic_obstacle_detection
