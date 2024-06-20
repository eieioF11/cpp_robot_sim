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
//
#include "model.hpp"

namespace cpp_robot_sim
{
  class simulator
  {
  private:
    std::chrono::system_clock::time_point now, old;
    std::function<state_t(state_t, control_t, double)> f_;
    double width_;
    double height_;
    std::vector<double> x_{0};
    std::vector<double> y_{0};
    matplotlibcpp17::pyplot::PyPlot plt_;

  public:
    state_t x_t;
    simulator(const matplotlibcpp17::pyplot::PyPlot &plt, std::function<state_t(state_t, control_t, double)> f, double width, double height)
    {
      f_ = f;
      this->now = std::chrono::system_clock::now();
      this->old = std::chrono::system_clock::now();
      width_ = width;
      height_ = height;
      plt_ = plt;
    }
    double deltaT()
    {
      this->now = std::chrono::system_clock::now();
      double dt = std::chrono::duration_cast<std::chrono::seconds>(this->now - this->old).count();
      this->old = this->now;
      return dt;
    }
    void reset()
    {
      x_t << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      x_.clear();
      y_.clear();
      x_.push_back(x_t(3));
      y_.push_back(x_t(4));
    }
    void update(control_t v_t, double dt)
    {
      x_t = f_(x_t, v_t, dt);
      x_.push_back(x_t(3));
      y_.push_back(x_t(4));
      std::cout << "control_input: " << v_t.transpose() << ", dt:" << dt << std::endl;
      std::cout << "x_t: " << x_t.transpose() << std::endl;
    }
    void draw(bool draw_trajectory = false)
    {
      double x = x_t(3);
      double y = x_t(4);
      double theta = x_t(5);
      double h_w = width_ / 2.;
      double h_h = height_ / 2.;
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
      plt_.plot(Args(x_data, y_data), Kwargs("color"_a = "black", "linewidth"_a = 1.0));
      plt_.plot(Args(std::vector<double>({x, pf(0)}), std::vector<double>({y, pf(1)})),
                Kwargs("color"_a = "black", "linewidth"_a = 2.0, "marker"_a = "o"));
      plt_.plot(Args(x, y), Kwargs("color"_a = "red", "linewidth"_a = 2.0, "marker"_a = "o"));
      if (draw_trajectory)
      {
        plt_.plot(Args(x_, y_), Kwargs("color"_a = "red", "linewidth"_a = 1.0));
      }
    }
  };
} // namespace cpp_robot_sim