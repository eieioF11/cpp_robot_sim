#pragma once
// std
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdio.h>
#include <unistd.h>
#include <vector>
// Eigen
#include <Eigen/Dense>
// matplotlibcpp17
#include <matplotlibcpp17/pyplot.h>

namespace cpp_robot_sim
{
  typedef Eigen::Matrix<double, 3, 1> vec3_t; // [vx,vy,omega]T
  typedef vec3_t control_t;                   // [vx,vy,omega]T
  typedef Eigen::Matrix<double, 6, 1> vec6_t; // [vx,vy,omega,x,y,theta]T
  typedef vec6_t state_t;                     // [vx,vy,omega,x,y,theta]T

  /*
    cpp_robot_sim::state_t f(cpp_robot_sim::state_t x_t, cpp_robot_sim::control_t v_t, double dt) {
      cpp_robot_sim::state_t x_next;
      Eigen::Matrix<double, 6, 6> A;
      A <<  1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
      Eigen::Matrix<double, 6, 3> B;
      B <<  0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;
      x_next = A*x_t + dt*B*v_t;
      return x_next;
    }
  */
  cpp_robot_sim::state_t omni_directional(cpp_robot_sim::state_t x_t, cpp_robot_sim::control_t v_t, double dt)
  {
    cpp_robot_sim::state_t x_next;
    Eigen::Matrix<double, 6, 6> A;
    A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix<double, 6, 3> B;
    B << 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
    x_next = A * x_t + dt * B * v_t;
    return x_next;
  }
  cpp_robot_sim::state_t two_wheels(cpp_robot_sim::state_t x_t, cpp_robot_sim::control_t v_t, double dt)
  {
    cpp_robot_sim::state_t x_next;
    Eigen::Matrix<double, 6, 6> A;
    A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix<double, 6, 3> B;
    B << 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        std::cos(x_t(5)), 0.0, 0.0,
        std::sin(x_t(5)), 0.0, 0.0,
        0.0, 0.0, 1.0;
    x_next = A * x_t + dt * B * v_t;
    return x_next;
  }
} // namespace cpp_robot_sim
