#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

#pragma once

class RotatingRectangle {
public:
  RotatingRectangle(Eigen::Vector2d center, double width, double height, double angle)
    : center_(center), width_(width), height_(height), angle_(angle) {}

  inline void set_rotation(double angle) { this->angle_ = angle; }

  Eigen::Vector2d rotate_point(const Eigen::Vector2d& point) const {
    double cos_theta = cos(angle_);
    double sin_theta = sin(angle_);
    return Eigen::Vector2d(cos_theta * point[0] - sin_theta * point[1],
                            sin_theta * point[0] + cos_theta * point[1]);
  }

  std::vector<Eigen::Vector2d> get_vertices() const {
    Eigen::Vector2d half_size(width_ / 2, height_ / 2);
    std::vector<Eigen::Vector2d> vertices = {
      Eigen::Vector2d( half_size[0],  half_size[1]),
      Eigen::Vector2d(-half_size[0],  half_size[1]),
      Eigen::Vector2d(-half_size[0], -half_size[1]),
      Eigen::Vector2d( half_size[0], -half_size[1])
    };

    for (auto& vertex : vertices) {
      vertex += center_;
      vertex = rotate_point(vertex - center_) + center_; // 先平移到原点再旋转
    }
    return vertices;
  }

  static std::pair<double, double> project(const std::vector<Eigen::Vector2d>& vertices, const Eigen::Vector2d& axis) {
    std::vector<double> projections;
    for (const auto& v : vertices) {
      projections.push_back(v.dot(axis));
    }
    return { *std::min_element(projections.begin(), projections.end()),
              *std::max_element(projections.begin(), projections.end()) };
  }

  static bool is_separating_axis(const std::vector<Eigen::Vector2d>& vertices1,
                                  const std::vector<Eigen::Vector2d>& vertices2,
                                  const Eigen::Vector2d& axis) {
    auto min_max1 = project(vertices1, axis);
    auto min_max2 = project(vertices2, axis);
    return min_max1.second < min_max2.first || min_max2.second < min_max1.first;
  }

  bool is_collision(const RotatingRectangle& other) const {
    auto vertices1 = get_vertices();
    auto vertices2 = other.get_vertices();

    std::vector<Eigen::Vector2d> axes;

    // 获取所有可能的分离轴
    for (int i = 0; i < 4; ++i) {
      Eigen::Vector2d edge = vertices1[i] - vertices1[(i + 1) % 4];
      Eigen::Vector2d axis(-edge[1], edge[0]);
      axes.push_back(axis.normalized());
    }
    for (int i = 0; i < 4; ++i) {
      Eigen::Vector2d edge = vertices2[i] - vertices2[(i + 1) % 4];
      Eigen::Vector2d axis(-edge[1], edge[0]);
      axes.push_back(axis.normalized());
    }

    // 检查所有轴上的投影是否重叠
    for (const auto& axis : axes) {
      if (is_separating_axis(vertices1, vertices2, axis)) {
          return false;
      }
    }
    return true;
  }

private:
  Eigen::Vector2d center_;
  double width_, height_, angle_;
};

// // 示例
// int main() {
//     RotatingRectangle rect1(Eigen::Vector2d(0, 0.1), 0.2, 0.1, 0);
//     RotatingRectangle rect2(Eigen::Vector2d(0, -0.1), 0.2, 0.1, 0);

//     bool collision = rect1.is_collision(rect2);
//     std::cout << "发生碰撞: " << collision << std::endl;

//     rect1.set_rotation(-3.14 / 4);
//     rect2.set_rotation(3.14 / 4);
//     collision = rect1.is_collision(rect2);
//     std::cout << "发生碰撞: " << collision << std::endl;

//     return 0;
// }
