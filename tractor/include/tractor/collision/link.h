// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/collision/shape.h>

#include <memory>

namespace tractor {

class CollisionLinkBase {
public:
  virtual void
  addConvexPolyhedron(const std::vector<Vector3<double>> &points,
                      const std::vector<Plane<double>> &planes) = 0;
  virtual void addSphere(const Vector3<double> &center, double radius) = 0;
  virtual void addCylinder(double radius, double length) = 0;
};

template <class Scalar> class CollisionLink : public CollisionLinkBase {
  std::string _name;
  std::vector<std::shared_ptr<const CollisionShape<Scalar>>> _shapes;

public:
  CollisionLink() {}
  CollisionLink(const std::string &name) : _name(name) {}
  const auto &shapes() const { return _shapes; }
  const std::string &name() const { return _name; }
  virtual void
  addConvexPolyhedron(const std::vector<Vector3<double>> &points,
                      const std::vector<Plane<double>> &planes) override {
    std::vector<Vector3<Scalar>> points2;
    for (auto &p : points) {
      points2.emplace_back(Scalar(p.x()), Scalar(p.y()), Scalar(p.z()));
    }
    std::vector<Plane<Scalar>> planes2;
    for (auto &plane : planes) {
      planes2.emplace_back(Vector3<Scalar>(Scalar(plane.normal().x()),
                                           Scalar(plane.normal().y()),
                                           Scalar(plane.normal().z())),
                           Scalar(plane.offset()));
    }
    _shapes.push_back(std::make_shared<ConvexPolyhedralCollisionShape<Scalar>>(
        points2, planes2));
  }
  virtual void addSphere(const Vector3<double> &center,
                         double radius) override {
    _shapes.push_back(std::make_shared<CollisionSphereShape<Scalar>>(
        Vector3<Scalar>(Scalar(center.x()), Scalar(center.y()),
                        Scalar(center.z())),
        Scalar(radius)));
  }
  virtual void addCylinder(double radius, double length) override {
    _shapes.push_back(std::make_shared<CollisionCylinderShape<Scalar>>(
        Scalar(radius), Scalar(length)));
  }
};

} // namespace tractor
