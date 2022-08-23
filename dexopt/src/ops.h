// (c) 2020-2021 Philipp Ruppel

#pragma once
#include <random>
#include <tractor/collision/query.h>
#include <tractor/core/eigen.h>
#include <tractor/core/operator.h>
#include <tractor/core/recorder.h>
#include <tractor/geometry/vector3.h>

namespace tractor {

template <class T> struct BarrierTypes {
  typedef Eigen::Matrix<T, 6, 1> GradientType;
  typedef Eigen::Matrix<T, 6, 6> HessianType;
  typedef Eigen::Matrix<T, 3, 1> Vector3;
  typedef Eigen::Matrix<T, 3, 3> Matrix3;
};

// ------------------------------------------

template <class T> inline void batchBackprop(T &da, const T &dx) { da = dx; }
template <class T, size_t S>
inline void batchBackprop(T &da, const Batch<T, S> &dx) {
  T rs = T(0);
  for (size_t i = 0; i < S; i++) {
    rs += dx[i];
  }
  da = rs;
}
TRACTOR_OP(batch, (const S &a, T &x), { x = T(a); })
TRACTOR_D(prepare, batch, (const S &a, const T &x), {})
TRACTOR_D(forward, batch, (const S &da, T &dx), { dx = T(da); })
TRACTOR_D(reverse, batch, (S & da, const T &dx), { batchBackprop(da, dx); })

// ------------------------------------------

template <class T>
auto dot4add(const T &a0, const T &a1, const T &a2, const T &a3, const T &b0,
             const T &b1, const T &b2, const T &b3, const T &add) {
  return a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3 + add;
}
TRACTOR_OP(dot4add,
           (const T &a0, const T &a1, const T &a2, const T &a3, const T &b0,
            const T &b1, const T &b2, const T &b3, const T &add),
           { return a0 * b0 + a1 * b1 + a2 * b2 + a3 * b3 + add; })
TRACTOR_D(prepare, dot4add,
          (const T &a0, const T &a1, const T &a2, const T &a3, const T &b0,
           const T &b1, const T &b2, const T &b3, const T &add, const T &x,
           std::array<T, 8> &p),
          {
            p[0] = a0;
            p[1] = a1;
            p[2] = a2;
            p[3] = a3;
            p[4] = b0;
            p[5] = b1;
            p[6] = b2;
            p[7] = b3;
          })
TRACTOR_D(forward, dot4add,
          (const std::array<T, 8> &p, const T &da0, const T &da1, const T &da2,
           const T &da3, const T &db0, const T &db1, const T &db2, const T &db3,
           const T &dadd, T &dx),
          {
            dx = dadd;
            dx += da0 * p[4] + p[0] * db0;
            dx += da1 * p[5] + p[1] * db1;
            dx += da2 * p[6] + p[2] * db2;
            dx += da3 * p[7] + p[3] * db3;
          })
TRACTOR_D(reverse, dot4add,
          (const std::array<T, 8> &p, T &da0, T &da1, T &da2, T &da3, T &db0,
           T &db1, T &db2, T &db3, T &dadd, const T &dx),
          {
            da0 = dx * p[4];
            db0 = dx * p[0];
            da1 = dx * p[5];
            db1 = dx * p[1];
            da2 = dx * p[6];
            db2 = dx * p[2];
            da3 = dx * p[7];
            db3 = dx * p[3];
            dadd = dx;
          })

// ------------------------------------------

TRACTOR_OP(acos, (const T &a),
           { return std::max(T(-1), std::min(T(1), T(std::acos(a)))); })
TRACTOR_D(prepare, acos, (const T &a, const T &x, T &p), {
  p = T(-1) / std::max(T(1e-9), T(std::sqrt(std::max(T(0), T(1) - a * a))));
})
TRACTOR_D(forward, acos, (const T &p, const T &da, T &dx), { dx = da * p; })
TRACTOR_D(reverse, acos, (const T &p, T &da, const T &dx), { da = dx * p; })

// ------------------------------------------

template <class T> auto relu(const T &a) { return std::max(T(0), a); }
TRACTOR_OP(relu, (const T &a), { return relu(a); })
TRACTOR_D_LOOP(forward, relu, (const T &a, const T &x, const T &da, T &dx),
               (a, x, da, dx), {
                 if (a >= T(0)) {
                   dx = da;
                 } else {
                   dx = T(0);
                 }
               })
TRACTOR_D_LOOP(reverse, relu, (const T &a, const T &x, T &da, const T &dx),
               (a, x, da, dx), {
                 if (a >= T(0)) {
                   da = dx;
                 } else {
                   da = T(0);
                 }
               })

// ------------------------------------------

template <class T> T add_random_normal(const T &a, const T &s) {
  static thread_local std::mt19937 rng{std::mt19937::result_type(rand())};
  std::normal_distribution<double> dist;
  return a + T(dist(rng)) * s;
}
template <class T, size_t S>
Batch<T, S> add_random_normal(const Batch<T, S> &a, const T &s) {
  Batch<T, S> ret;
  for (size_t i = 0; i < S; i++) {
    ret[i] = add_random_normal(a[i], s);
  }
  return ret;
}
TRACTOR_OP(add_random_normal, (const T &a, const S &s),
           { return add_random_normal(a, s); })
TRACTOR_D(prepare, add_random_normal, (const T &a, const S &s, const T &x), {})
TRACTOR_D(forward, add_random_normal, (const T &da, const S &ds, T &dx),
          { dx = da; })
TRACTOR_D(reverse, add_random_normal, (T & da, S &ds, const T &dx), {
  da = dx;
  ds = S(0);
})

// ------------------------------------------

template <class T> T add_random_uniform(const T &a, const T &l, const T &h) {
  static thread_local std::mt19937 rng{std::mt19937::result_type(rand())};
  std::uniform_real_distribution<double> dist(l, h);
  return a + T(dist(rng));
}
template <class T, size_t S>
Batch<T, S> add_random_uniform(const Batch<T, S> &a, const T &l, const T &h) {
  Batch<T, S> ret;
  for (size_t i = 0; i < S; i++) {
    ret[i] = add_random_uniform(a[i], l, h);
  }
  return ret;
}
TRACTOR_OP(add_random_uniform, (const T &a, const S &l, const S &h),
           { return add_random_uniform(a, l, h); })
TRACTOR_D(prepare, add_random_uniform,
          (const T &a, const S &l, const S &h, const T &x), {})
TRACTOR_D(forward, add_random_uniform,
          (const T &da, const S &dl, const S &dh, T &dx), { dx = da; })
TRACTOR_D(reverse, add_random_uniform, (T & da, S &dl, S &dh, const T &dx), {
  da = dx;
  dl = S(0);
  dh = S(0);
})

// ------------------------------------------

template <class T> inline void dropout2(const T &a, const T &b, T &x, T &y) {
  static thread_local std::mt19937 rng{std::mt19937::result_type(rand())};
  std::uniform_real_distribution<double> dist;
  y = (dist(rng) < b) ? T(0) : T(1.0 / (1.0 - b));
  x = a * y;
}
template <class T, size_t S>
inline void dropout2(const Batch<T, S> &a, const T &b, Batch<T, S> &x,
                     Batch<T, S> &y) {
  for (size_t i = 0; i < S; i++) {
    dropout2(a[i], b, x[i], y[i]);
  }
}
TRACTOR_OP(dropout2, (const T &a, const S &b, T &x, T &y),
           { dropout2(a, b, x, y); })
TRACTOR_D(prepare, dropout2,
          (const T &a, const S &b, const T &x, const T &y, T &s), { s = y; })
TRACTOR_D(forward, dropout2,
          (const T &s, const T &da, const S &db, T &dx, T &dy), {
            dx = da * s;
            dy = T(0);
          })
TRACTOR_D(reverse, dropout2,
          (const T &s, T &da, S &db, const T &dx, const T &dy), {
            da = dx * s;
            db = S(0);
          })

template <class A, class B> auto dropout(const A &a, const B &b) {
  A x, y;
  dropout2(a, b, x, y);
  return x;
}

// ------------------------------------------

template <class T>
static void collision_project_2(const Vector3<T> &point,
                                const uint64_t &shape_id, Vector3<T> &normal,
                                T &distance) {
  auto &shape = *(CollisionShape<T> *)shape_id;
  shape.project(point, normal, distance);
}

template <class T, size_t S>
static void
collision_project_2(const Vector3<Batch<T, S>> &point, const uint64_t &shape_id,
                    Vector3<Batch<T, S>> &normal, Batch<T, S> &distance) {
  auto &shape = *(CollisionShape<T> *)shape_id;
  for (size_t i = 0; i < S; i++) {
    Vector3<T> n;
    shape.project(Vector3<T>(point.x()[i], point.y()[i], point.z()[i]), n,
                  distance[i]);
    normal.x()[i] = n.x();
    normal.y()[i] = n.y();
    normal.z()[i] = n.z();
  }
}

TRACTOR_OP(collision_project_2,
           (const Vector3<T> &point, const uint64_t &shape_id,
            Vector3<T> &normal, T &distance),
           { collision_project_2(point, shape_id, normal, distance); })
TRACTOR_D(prepare, collision_project_2,
          (const Vector3<T> &point, const uint64_t &shape_id,
           const Vector3<T> &normal, const T &distance, Vector3<T> &n),
          { n = normal; })
TRACTOR_D(forward, collision_project_2,
          (const Vector3<T> &n, const Vector3<T> &point,
           const uint64_t &shape_id, Vector3<T> &normal, T &distance),
          {
            normal.setZero();
            distance = -dot(n, point);
          })
TRACTOR_D(reverse, collision_project_2,
          (const Vector3<T> &n, Vector3<T> &point, uint64_t &shape_id,
           const Vector3<T> &normal, const T &distance),
          {
            point = n * -distance;
            shape_id = 0;
          })

// ------------------------------------------

template <class T>
static void collision_project(const Vector3<T> &point, const uint64_t &shape_id,
                              Vector3<T> &normal, T &distance) {
  // auto &shape = *(CollisionShape<T> *)shape_id;
  // shape.project(point, normal, distance);
  throw std::runtime_error("NYI collision_project");
}
TRACTOR_OP(collision_project,
           (const Vector3<T> &point, const uint64_t &shape_id,
            Vector3<T> &normal, T &distance),
           { collision_project(point, shape_id, normal, distance); })
TRACTOR_D(prepare, collision_project,
          (const Vector3<T> &point, const uint64_t &shape_id,
           const Vector3<T> &normal, const T &distance),
          {})
TRACTOR_D(forward, collision_project,
          (const Vector3<T> &point, const uint64_t &shape_id,
           Vector3<T> &normal, T &distance),
          {
            normal.setZero();
            distance = T(0);
          })
TRACTOR_D(reverse, collision_project,
          (Vector3<T> & point, uint64_t &shape_id, const Vector3<T> &normal,
           const T &distance),
          {
            point.setZero();
            shape_id = 0;
          })

// ------------------------------------------

template <class T>
void collision_axes(const Pose<T> &pose_a, const Pose<T> &pose_b,
                    const uint64_t &shape_pair_id, Vector3<T> &point_a,
                    Vector3<T> &point_b, Vector3<T> &axis, Vector3<T> &local_a,
                    Vector3<T> &local_b) {

  auto &shape_pair = *(ShapeCollisionPair<T> *)shape_pair_id;

  shape_pair.update(pose_a, pose_b);

  point_a = shape_pair.pointA();
  point_b = shape_pair.pointB();

  axis = shape_pair.normal();

  local_a = pose_a.inverse() * shape_pair.pointA();
  local_b = pose_b.inverse() * shape_pair.pointB();
}

template <class T, size_t S>
void collision_axes(const Pose<Batch<T, S>> &pose_a,
                    const Pose<Batch<T, S>> &pose_b,
                    const uint64_t &shape_pair_id,
                    Vector3<Batch<T, S>> &point_a,
                    Vector3<Batch<T, S>> &point_b, Vector3<Batch<T, S>> &axis,
                    Vector3<Batch<T, S>> &local_a,
                    Vector3<Batch<T, S>> &local_b) {

  auto &shape_pair = *(ShapeCollisionPair<T> *)shape_pair_id;

  for (size_t i = 0; i < S; i++) {

    shape_pair.update(indexBatch(pose_a, i), indexBatch(pose_b, i));

    point_a.x()[i] = shape_pair.pointA().x();
    point_a.y()[i] = shape_pair.pointA().y();
    point_a.z()[i] = shape_pair.pointA().z();

    point_b.x()[i] = shape_pair.pointB().x();
    point_b.y()[i] = shape_pair.pointB().y();
    point_b.z()[i] = shape_pair.pointB().z();

    axis.x()[i] = shape_pair.normal().x();
    axis.y()[i] = shape_pair.normal().y();
    axis.z()[i] = shape_pair.normal().z();
  }

  local_a = pose_a.inverse() * point_a;
  local_b = pose_b.inverse() * point_b;
}

TRACTOR_OP(collision_axes,
           (const Pose<T> &pose_a, const Pose<T> &pose_b,
            const uint64_t &shape_pair_id, Vector3<T> &point_a,
            Vector3<T> &point_b, Vector3<T> &axis, Vector3<T> &local_a,
            Vector3<T> &local_b),
           {
             collision_axes(pose_a, pose_b, shape_pair_id, point_a, point_b,
                            axis, local_a, local_b);
           })
TRACTOR_D(prepare, collision_axes,
          (const Pose<T> &pose_a, const Pose<T> &pose_b,
           const uint64_t &shape_pair_id, const Vector3<T> &point_a,
           const Vector3<T> &point_b, const Vector3<T> &axis,
           const Vector3<T> &local_a, const Vector3<T> &local_b),
          {})
TRACTOR_D(forward, collision_axes,
          (const Twist<T> &pose_a, const Twist<T> &pose_b,
           const uint64_t &shape_pair_id, Vector3<T> &point_a,
           Vector3<T> &point_b, Vector3<T> &axis, Vector3<T> &local_a,
           Vector3<T> &local_b),
          {
            point_a.setZero();
            point_b.setZero();
            axis.setZero();
            local_a.setZero();
            local_b.setZero();
          })
TRACTOR_D(reverse, collision_axes,
          (Twist<T> & pose_a, Twist<T> &pose_b, uint64_t &shape_pair_id,
           const Vector3<T> &point_a, const Vector3<T> &point_b,
           const Vector3<T> &axis, const Vector3<T> &local_a,
           const Vector3<T> &local_b),
          {
            pose_a.setZero();
            pose_b.setZero();
            shape_pair_id = 0;
          })

// ------------------------------------------

TRACTOR_OP(capture_constraint, (const Vector3<T> &a, const uint64_t &shape_id),
           { return Vector3<T>::Zero(); })
TRACTOR_D(prepare, capture_constraint,
          (const Vector3<T> &a, const uint64_t &shape_id, const Vector3<T> &x),
          {})
TRACTOR_D(forward, capture_constraint,
          (const Vector3<T> &a, const uint64_t &shape_id, Vector3<T> &x),
          { x.setZero(); })
TRACTOR_D(reverse, capture_constraint,
          (Vector3<T> & a, uint64_t &shape_id, const Vector3<T> &x), {
            a.setZero();
            shape_id = 0;
          })
TRACTOR_D(project, capture_constraint,
          (const Vector3<T> &a, const uint64_t &shape_id, const Vector3<T> &da,
           const T &padding, Vector3<T> &dx),
          {
            dx = da;
            // dx = shape.center() - a;
          })
TRACTOR_D(barrier_init, capture_constraint,
          (const Vector3<T> &a, const uint64_t &shape_id, const Vector3<T> &da,
           Eigen::Matrix<T, 3, 1, Eigen::DontAlign> &dx,
           Eigen::Matrix<T, 3, 3, Eigen::DontAlign> &ddx),
          {
            throw std::runtime_error("NYI capture_constraint");
            /*
          auto &shape = *(CollisionShape<T> *)shape_id;
          BarrierTypes<T>::Vector3 rdx;
          BarrierTypes<T>::Matrix3 rddx;
          shape.capture(a + da, rdx, rddx);
          dx = rdx;
          ddx = rddx;
          */
          })
TRACTOR_D(barrier_step, capture_constraint,
          (const Eigen::Matrix<T, 3, 3, Eigen::DontAlign> &dda,
           const Eigen::Matrix<T, 3, 1, Eigen::DontAlign> &da,
           Eigen::Matrix<T, 3, 1, Eigen::DontAlign> &dx),
          { dx = dda * da; })
TRACTOR_D(barrier_diagonal, capture_constraint,
          (const Eigen::Matrix<T, 3, 3, Eigen::DontAlign> &dda,
           Eigen::Matrix<T, 3, 1, Eigen::DontAlign> &ddx),
          { ddx = dda.diagonal(); })

// ------------------------------------------

TRACTOR_OP(simple_collision_constraint,
           (const Vector3<T> &a, const Quaternion<T> &qa,
            const Quaternion<T> &qb, const uint64_t &pair),
           { return Vector3<T>::Zero(); })
TRACTOR_D(prepare, simple_collision_constraint,
          (const Vector3<T> &a, const Quaternion<T> &qa,
           const Quaternion<T> &qb, const uint64_t &pair, const Vector3<T> &x),
          {})
TRACTOR_D(forward, simple_collision_constraint,
          (const Vector3<T> &a, const Vector3<T> &qa, const Vector3<T> &qb,
           const uint64_t &pair, Vector3<T> &x),
          { // x = T(0);
            x.setZero();
          })
TRACTOR_D(reverse, simple_collision_constraint,
          (Vector3<T> & a, Vector3<T> &qa, Vector3<T> &qb, uint64_t &pair,
           const Vector3<T> &x),
          {
            a.setZero();
            qa.setZero();
            qb.setZero();
            pair = 0;
          })
TRACTOR_D(project, simple_collision_constraint,
          (const Vector3<T> &a, const Quaternion<T> &qa,
           const Quaternion<T> &qb, const uint64_t &pair, const Vector3<T> &da,
           const T &padding, Vector3<T> &dx),
          { throw std::runtime_error("NYI simple_collision_constraint"); })
template <class T> struct SimpleCollisionConstraintBarrier {
  Eigen::Matrix<T, 6, 6, Eigen::DontAlign> hessian;
  uint64_t test = 0;
};
TRACTOR_D(barrier_init, simple_collision_constraint,
          (const Vector3<T> &a, const Quaternion<T> &qa,
           const Quaternion<T> &qb, const uint64_t &pair, const Vector3<T> &da,
           Eigen::Matrix<T, 3, 1, Eigen::DontAlign> &dx,
           SimpleCollisionConstraintBarrier<T> &ddx),
          { throw std::runtime_error("NYI simple_collision_constraint"); })
TRACTOR_D(barrier_step, simple_collision_constraint,
          (const SimpleCollisionConstraintBarrier<T> &dda,
           // const Twist<T> &da, Twist<T> &dx
           const Eigen::Matrix<T, 3, 1, Eigen::DontAlign> &da,
           Eigen::Matrix<T, 3, 1, Eigen::DontAlign> &dx),
          {
            // dx.setZero();

            if (dda.test != 123456789) {
              throw 1;
            }

            dx = dda.hessian.block(0, 0, 3, 3) * da;
            // dx = da;
            // dx = da * T(1);
          })
TRACTOR_D(barrier_diagonal, simple_collision_constraint,
          (const SimpleCollisionConstraintBarrier<T> &dda,
           Eigen::Matrix<T, 3, 1, Eigen::DontAlign> &ddx),
          {
            if (dda.test != 123456789) {
              throw 1;
            }

            ddx = dda.hessian.diagonal().head(3);
            // ddx = Twist<T>(Vector3<T>(1, 1, 1), Vector3<T>(1, 1, 1));
            // ddx = Twist<T>(Vector3<T>(0, 0, 0), Vector3<T>(0, 0, 0));
          })

// ------------------------------------------

#if 1
TRACTOR_OP(collision_constraint, (const Pose<T> &a, const uint64_t &pair),
           { return T(0); })
TRACTOR_D(prepare, collision_constraint,
          (const Pose<T> &a, const uint64_t &pair, const T &x), {})
TRACTOR_D(forward, collision_constraint,
          (const Twist<T> &a, const uint64_t &pair, T &x), { x = T(0); })
TRACTOR_D(reverse, collision_constraint,
          (Twist<T> & a, uint64_t &pair, const T &x), {
            a.setZero();
            pair = 0;
          })
TRACTOR_D(project, collision_constraint,
          (const Pose<T> &a, const uint64_t &pair, const Twist<T> &da,
           const T &padding, Twist<T> &dx),
          { throw std::runtime_error("NYI collision_constraint"); })
TRACTOR_D(barrier_init, collision_constraint,
          (const Pose<T> &a, const uint64_t &pair, const Twist<T> &da,
           Twist<T> &dx, Eigen::Matrix<T, 6, 6, Eigen::DontAlign> &ddx),
          { throw std::runtime_error("NYI collision_constraint"); })
TRACTOR_D(barrier_step, collision_constraint,
          (const Eigen::Matrix<T, 6, 6, Eigen::DontAlign> &dda,
           const Eigen::Matrix<T, 6, 1, Eigen::DontAlign> &da,
           Eigen::Matrix<T, 6, 1, Eigen::DontAlign> &dx),
          {
            TRACTOR_PROFILER("collision constraint step");
            dx = dda * da;
          })
TRACTOR_D(barrier_diagonal, collision_constraint,
          (const Eigen::Matrix<T, 6, 6, Eigen::DontAlign> &dda,
           Eigen::Matrix<T, 6, 1, Eigen::DontAlign> &ddx),
          {
            TRACTOR_PROFILER("collision constraint diagonal");
            ddx = dda.diagonal();
          })
#endif

TRACTOR_OP(sphere_collision_constraint, (const Vector3<T> &a, const T &radius),
           { return T(0); })
TRACTOR_D(prepare, sphere_collision_constraint,
          (const Vector3<T> &a, const T &radius, const T &x), {})
TRACTOR_D(forward, sphere_collision_constraint,
          (const Vector3<T> &a, const T &radius, T &x), { x = T(0); })
TRACTOR_D(reverse, sphere_collision_constraint,
          (Vector3<T> & a, T &radius, const T &x), {
            a.setZero();
            radius = T(0);
          })
TRACTOR_D(project, sphere_collision_constraint,
          (const Vector3<T> &a, const T &radius, const Vector3<T> &da,
           const T &padding, Vector3<T> &dx),
          {
            Vector3<T> normal = normalized(a);
            dx = da + normal * std::max(T(0), radius - dot(normal, a + da));
          })
TRACTOR_D(barrier_init, sphere_collision_constraint,
          (const Vector3<T> &a, const T &radius, const Vector3<T> &da,
           Vector3<T> &dx, Vector3<T> &ddx),
          {
            Vector3<T> normal = normalized(a);
            T d = T(-1) / std::max(T(0), dot(normal, a + da) - radius);
            dx = normal * d;
            ddx = normal * (d * d);
            ddx = normal * d;
          })
TRACTOR_D(barrier_step, sphere_collision_constraint,
          (const Vector3<T> &dda, const Vector3<T> &da, Vector3<T> &dx),
          { dx = dda * dot(dda, da); })
TRACTOR_D(barrier_diagonal, sphere_collision_constraint,
          (const Vector3<T> &dda, Vector3<T> &ddx), {
            ddx.x() = dda.x() * dda.x();
            ddx.y() = dda.y() * dda.y();
            ddx.z() = dda.z() * dda.z();
          })

} // namespace tractor
