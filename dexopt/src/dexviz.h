// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <ros/ros.h>

#include <tractor/tractor.h>

#include "common.h"

namespace tractor {

class DexViz {

  visualization_msgs::Marker _point_marker;
  visualization_msgs::Marker _line_marker;

  static auto makeColor(double r, double g, double b, double a = 1) {
    std_msgs::ColorRGBA ret;
    ret.r = r;
    ret.g = g;
    ret.b = b;
    ret.a = a;
    return ret;
  };

  static Eigen::Vector3d hue2color(double h) {
    h -= floor(h);
    h *= 6;
    if (h < 1)
      return Eigen::Vector3d(1, h, 0);
    else if (h < 2)
      return Eigen::Vector3d(2 - h, 1, 0);
    else if (h < 3)
      return Eigen::Vector3d(0, 1, h - 2);
    else if (h < 4)
      return Eigen::Vector3d(0, 4 - h, 1);
    else if (h < 5)
      return Eigen::Vector3d(h - 4, 0, 1);
    else
      return Eigen::Vector3d(1, 0, 6 - h);
  }

public:
  void clear() {

    _point_marker = visualization_msgs::Marker();
    _point_marker.ns = "points";
    _point_marker.color.r = 1;
    _point_marker.color.g = 1;
    _point_marker.color.b = 1;
    _point_marker.color.a = 1;
    _point_marker.scale.x = 0.01;
    _point_marker.type = visualization_msgs::Marker::POINTS;

    _line_marker = visualization_msgs::Marker();
    _line_marker.ns = "lines";
    _line_marker.color.r = 1;
    _line_marker.color.g = 1;
    _line_marker.color.b = 1;
    _line_marker.color.a = 1;
    _line_marker.scale.x = 0.0015;
    _line_marker.type = visualization_msgs::Marker::LINE_LIST;
  }

  auto finish() const {

    auto line_marker = _line_marker;

    {
      size_t n = line_marker.points.size();
      line_marker.colors.resize(n);
      for (size_t i = 0; i < n; i++) {
        auto color = hue2color(i * 1.0 / n);
        line_marker.colors[i].r = color.x();
        line_marker.colors[i].g = color.y();
        line_marker.colors[i].b = color.z();
        line_marker.colors[i].a = 1;
      }
    }

    visualization_msgs::MarkerArray ret;
    ret.markers.push_back(_point_marker);
    ret.markers.push_back(line_marker);

    return ret;
  }

  template <class Vector3>
  void visualizeContact(const Vector3 &point, const Vector3 &normal,
                        const Vector3 &force, size_t i) {

    // static std::vector<Eigen::Vector3d> colors = {
    //     {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}, {0, 1, 1},
    // };
    // auto color = colors[i];

    // hue2color

    double len = 2;
    // double len = 0.5;

    if ((norm(force) * len) < 0.01) {
      return;
    }

    _line_marker.points.emplace_back();
    _line_marker.points.back().x = point.x() - force.x() * len;
    _line_marker.points.back().y = point.y() - force.y() * len;
    _line_marker.points.back().z = point.z() - force.z() * len;

    _line_marker.points.emplace_back();
    _line_marker.points.back().x = point.x();
    _line_marker.points.back().y = point.y();
    _line_marker.points.back().z = point.z();

    //   for (size_t j = 0; j < 2; j++) {
    //     _line_marker.colors.emplace_back();
    //     _line_marker.colors.back().a = 1;
    //     _line_marker.colors.back().g = 1;
    //     if (i == 0) {
    //       _line_marker.colors.back().r = 1;
    //     } else {
    //       _line_marker.colors.back().b = 1;
    //     }
    //   }

    // for (size_t j = 0; j < 2; j++) {
    //   _line_marker.colors.emplace_back();
    //   _line_marker.colors.back().a = 1;
    //   _line_marker.colors.back().r = color.x();
    //   _line_marker.colors.back().g = color.y();
    //   _line_marker.colors.back().b = color.z();
    // }

    // {
    //   _point_marker.points.emplace_back();
    //   _point_marker.points.back().x = point.x();
    //   _point_marker.points.back().y = point.y();
    //   _point_marker.points.back().z = point.z();
    // }
  }
};

} // namespace tractor
