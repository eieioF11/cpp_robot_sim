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
// OpenMP
#include <omp.h>
//
#include "model.hpp"

namespace cpp_robot_sim {
  class simulator {
  private:
    double pre_time_;
    std::function<state_t(state_t, control_t, double)> f_;
    double width_;
    double height_;
    std::vector<double> x_{0};
    std::vector<double> y_{0};

  public:
    state_t x_t;
    simulator(std::function<state_t(state_t, control_t, double)> f, double width, double height) {
      f_        = f;
      width_    = width;
      height_   = height;
      pre_time_ = omp_get_wtime();
    }
    double deltaT() {
      double now_time = omp_get_wtime();
      double dt       = now_time - pre_time_;
      pre_time_       = now_time;
      return dt;
    }
    void reset() {
      x_t << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      x_.clear();
      y_.clear();
      x_.push_back(x_t(3));
      y_.push_back(x_t(4));
    }
    void update(control_t v_t, double dt) {
      x_t = f_(x_t, v_t, dt);
      x_.push_back(x_t(3));
      y_.push_back(x_t(4));
      std::cout << "control_input: " << v_t.transpose() << ", dt:" << dt << std::endl;
      std::cout << "x_t: " << x_t.transpose() << std::endl;
    }
    void draw(matplotlibcpp17::pyplot::PyPlot& plt, bool draw_trajectory = false) {
      double x     = x_t(3);
      double y     = x_t(4);
      double theta = x_t(5);
      double h_w   = width_ / 2.;
      double h_h   = height_ / 2.;
      Eigen::Matrix<double, 2, 1> po, p1, p2, p3, p4, pf;
      po << x, y; // ロボット中心座標
      p1 << h_w, h_h;
      p2 << -h_w, h_h;
      p3 << -h_w, -h_h;
      p4 << h_w, -h_h;
      pf << h_w, 0.; // ロボット前方座標
      // 回転行列
      Eigen::Matrix<double, 2, 2> rot;
      rot << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
      // 回転後の座標
      p1 = po + rot * p1;
      p2 = po + rot * p2;
      p3 = po + rot * p3;
      p4 = po + rot * p4;
      pf = po + rot * pf;
      // 描画
      std::vector<double> x_data = {p1(0), p2(0), p3(0), p4(0), p1(0)};
      std::vector<double> y_data = {p1(1), p2(1), p3(1), p4(1), p1(1)};
      plt.plot(Args(x_data, y_data), Kwargs("color"_a = "black", "linewidth"_a = 1.0));
      plt.plot(Args(std::vector<double>({x, pf(0)}), std::vector<double>({y, pf(1)})),
               Kwargs("color"_a = "black", "linewidth"_a = 2.0, "marker"_a = "o"));
      plt.plot(Args(x, y), Kwargs("color"_a = "red", "linewidth"_a = 2.0, "marker"_a = "o"));
      if (draw_trajectory) {
        plt.plot(Args(x_, y_), Kwargs("color"_a = "red", "linewidth"_a = 1.0));
      }
    }
  };
} // namespace cpp_robot_sim