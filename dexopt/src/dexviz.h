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

public:
  void clear() {

    _point_marker = visualization_msgs::Marker();
    _point_marker.ns = "points";
    _point_marker.color.r = 1;
    _point_marker.color.g = 1;
    _point_marker.color.b = 1;
    _point_marker.color.a = 1;
    _point_marker.scale.x = 0.001;
    _point_marker.type = visualization_msgs::Marker::POINTS;

    _line_marker = visualization_msgs::Marker();
    _line_marker.ns = "lines";
    _line_marker.color.r = 1;
    _line_marker.color.g = 1;
    _line_marker.color.b = 1;
    _line_marker.color.a = 1;
    _line_marker.scale.x = 0.0002;
    _line_marker.type = visualization_msgs::Marker::LINE_LIST;
  }

  auto finish() const {
    visualization_msgs::MarkerArray ret;
    ret.markers.push_back(_point_marker);
    ret.markers.push_back(_line_marker);
    return ret;
  }

  template <class Vector3>
  void visualizeContact(const Vector3 &point, const Vector3 &normal,
                        const Vector3 &force, size_t i) {

    {
      double len = 1;

      _line_marker.points.emplace_back();
      _line_marker.points.back().x = point.x() - force.x() * len;
      _line_marker.points.back().y = point.y() - force.y() * len;
      _line_marker.points.back().z = point.z() - force.z() * len;

      _line_marker.points.emplace_back();
      _line_marker.points.back().x = point.x();
      _line_marker.points.back().y = point.y();
      _line_marker.points.back().z = point.z();

      for (size_t j = 0; j < 2; j++) {
        _line_marker.colors.emplace_back();
        _line_marker.colors.back().a = 1;
        _line_marker.colors.back().g = 1;
        if (i == 0) {
          _line_marker.colors.back().r = 1;
        } else {
          _line_marker.colors.back().b = 1;
        }
      }
    }

    {
      _point_marker.points.emplace_back();
      _point_marker.points.back().x = point.x();
      _point_marker.points.back().y = point.y();
      _point_marker.points.back().z = point.z();
    }
  }
};

} // namespace tractor
