// (c) 2020-2021 Philipp Ruppel

#include <tractor/collision/robot.h>

#include <geometric_shapes/mesh_operations.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <LinearMath/btConvexHullComputer.h>
#include <LinearMath/btGeometryUtil.h>

namespace tractor {

Vector3<double> convertBullet(const btVector3 &v) {
  return Vector3<double>(v.x(), v.y(), v.z());
}

class ConvexHull {
  std::vector<Vector3<double>> _vertices;
  std::vector<Plane<double>> _planes;

public:
  auto &vertices() const { return _vertices; }
  auto &planes() const { return _planes; }
  void build(std::vector<Vector3<double>> points) {

    auto support = [&](const Vector3<double> &direction) {
      Vector3<double> ret = points.front();
      for (auto &p : points) {
        if (dot(p, direction) > dot(ret, direction)) {
          ret = p;
        }
      }
      return ret;
    };

    // if (points.size() > 10) {
    if (0) {

      static AlignedStdVector<Vector3<double>> directions = []() {
        AlignedStdVector<Vector3<double>> directions;
        for (int x = -1; x <= 1; x++) {
          for (int y = -1; y <= 1; y++) {
            for (int z = -1; z <= 1; z++) {
              if (x == 0 && y == 0 && z == 0) {
                continue;
              }
              directions.push_back(normalized(Vector3<double>(x, y, z)));
            }
          }
        }
        return directions;
      }();

      std::vector<Plane<double>> planes;
      for (auto &dir : directions) {
        planes.push_back(Plane<double>(dir, support(dir)));
      }
      // for (auto &p : planes) {
      //    std::cout << "plane " << p << std::endl;
      //}

      btAlignedObjectArray<btVector3> bt_plane_equations;
      for (auto &plane : planes) {
        btVector3 v;
        v[0] = plane.normal().x();
        v[1] = plane.normal().y();
        v[2] = plane.normal().z();
        v[3] = plane.offset();
        bt_plane_equations.push_back(v);
      }
      btAlignedObjectArray<btVector3> bt_vertices;
      btGeometryUtil::getVerticesFromPlaneEquations(bt_plane_equations,
                                                    bt_vertices);

      points.clear();
      for (size_t i = 0; i < bt_vertices.size(); i++) {
        auto &p = bt_vertices[i];
        points.emplace_back(p.x(), p.y(), p.z());
      }

      // for (auto &p : points) {
      //    std::cout << "point " << p << std::endl;
      //}
      // throw 0;
    }

    btConvexHullComputer hull_computer;
    hull_computer.compute(points[0].data(), sizeof(points[0]), points.size(),
                          btScalar(0), btScalar(0));

    _vertices.clear();
    for (size_t i = 0; i < hull_computer.vertices.size(); i++) {
      auto &v = hull_computer.vertices[i];
      _vertices.emplace_back(v.x(), v.y(), v.z());
      // std::cout << "hull vertex " << _vertices.back() << std::endl;
    }

    _planes.clear();
    for (size_t face_index = 0; face_index < hull_computer.faces.size();
         face_index++) {
      auto *edge1 = &hull_computer.edges[hull_computer.faces[face_index]];
      auto *edge2 = edge1->getNextEdgeOfFace();
      auto v0 = convertBullet(hull_computer.vertices[edge1->getSourceVertex()]);
      auto v1 = convertBullet(hull_computer.vertices[edge1->getTargetVertex()]);
      auto v2 = convertBullet(hull_computer.vertices[edge2->getTargetVertex()]);
      _planes.emplace_back(normalized(cross(v1 - v0, v2 - v0)),
                           (v0 + v1 + v2) * (1.0 / 3.0));
      // std::cout << "hull plane " << _planes.back() << std::endl;
    }
    // getchar();

    // std::cout << "vertices:" << _vertices.size() << " planes:" <<
    // _planes.size()
    //          << std::endl;
    // getchar();
  }
};

static void _loadCollisionRobotImpl(
    CollisionRobotBase *robot,
    const std::shared_ptr<CollisionLinkBase> &collision_link,
    const Eigen::Isometry3d &transform,
    const moveit::core::LinkModel *link_model, bool merge_fixed_links) {

  auto &shapes = link_model->getShapes();
  auto &origins = link_model->getCollisionOriginTransforms();
  for (size_t shape_index = 0; shape_index < shapes.size(); shape_index++) {
    auto &shape = shapes[shape_index];
    auto &shape_origin = origins[shape_index];
    Eigen::Isometry3d shape_pose((transform * shape_origin).matrix());

#if 1
    if (auto *sphere = dynamic_cast<const shapes::Sphere *>(shape.get())) {
      Eigen::Vector3d p = shape_pose.translation();
      collision_link->addSphere(Vector3<double>(p.x(), p.y(), p.z()),
                                sphere->radius);
    } else
#endif

#if 1
        if (auto *cylinder =
                dynamic_cast<const shapes::Cylinder *>(shape.get())) {
      collision_link->addCylinder(cylinder->radius, cylinder->length);
    } else
#endif

    {

      const shapes::Mesh *mesh =
          dynamic_cast<const shapes::Mesh *>(shape.get());
      const shapes::Mesh *mesh_cleanup = nullptr;
      if (!mesh) {
        mesh_cleanup = mesh = shapes::createMeshFromShape(shape.get());
      }
      std::vector<Vector3<double>> vertices;
      for (size_t vertex_index = 0; vertex_index < mesh->vertex_count;
           vertex_index++) {
        Eigen::Vector3d vertex(mesh->vertices[vertex_index * 3 + 0],
                               mesh->vertices[vertex_index * 3 + 1],
                               mesh->vertices[vertex_index * 3 + 2]);
        vertex = shape_pose * vertex;
        vertices.emplace_back(vertex.x(), vertex.y(), vertex.z());
      }
      ConvexHull hull;
      hull.build(vertices);
      if (!vertices.empty()) {
        collision_link->addConvexPolyhedron(hull.vertices(), hull.planes());
      }
      delete mesh_cleanup;
    }
  }

  for (auto *child_joint : link_model->getChildJointModels()) {
    auto *child_link = child_joint->getChildLinkModel();
    auto lnk = robot->createLink(child_joint->getChildLinkModel()->getName());
    if (merge_fixed_links &&
        child_joint->getType() == moveit::core::JointModel::FIXED) {
      _loadCollisionRobotImpl(
          robot, collision_link,
          Eigen::Isometry3d(
              (transform * child_link->getJointOriginTransform()).matrix()),
          child_link, merge_fixed_links);
    } else {
      _loadCollisionRobotImpl(robot, lnk, Eigen::Isometry3d::Identity(),
                              child_link, merge_fixed_links);
    }
  }
}

void CollisionRobotBase::_load(const moveit::core::RobotModel &robot_model,
                               bool merge_fixed_links) {
  _loadCollisionRobotImpl(
      this,
      createLink(robot_model.getRootJoint()->getChildLinkModel()->getName()),
      Eigen::Isometry3d::Identity(), robot_model.getRootLink(),
      merge_fixed_links);
}

} // namespace tractor
