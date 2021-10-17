// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <tractor/core/operator.h>
#include <tractor/geometry/matrix3.h>
#include <tractor/geometry/pose.h>
#include <tractor/geometry/quaternion.h>
#include <tractor/geometry/twist.h>
#include <tractor/geometry/vector3.h>

namespace tractor {

TRACTOR_VAR_OP(dot)
TRACTOR_VAR_OP(cross)

// ---------------------------------------------------------

TRACTOR_OP_T(fg_mat3, move, (const Matrix3<T> &v), { return Matrix3<T>(v); })
TRACTOR_D_T(prepare, fg_mat3, move, (const Matrix3<T> &a, const Matrix3<T> &x),
            {})
TRACTOR_D_T(forward, fg_mat3, move, (const Matrix3<T> &da, Matrix3<T> &dx),
            { dx = da; })
TRACTOR_D_T(reverse, fg_mat3, move, (Matrix3<T> & da, const Matrix3<T> &dx),
            { da = dx; })

TRACTOR_OP_T(fg_mat3_vec3, mul, (const Matrix3<T> &a, const Vector3<T> &b), {
  // return a * b;
  Vector3<T> x = Vector3<T>::Zero();
  for (size_t row = 0; row < 3; row++) {
    for (size_t col = 0; col < 3; col++) {
      x[row] += a(row, col) * b[col];
    }
  }
  return x;
})
TRACTOR_D_T(forward, fg_mat3_vec3, mul,
            (const Matrix3<T> &pa, const Vector3<T> &pb, const Matrix3<T> &px,
             const Matrix3<T> &da, const Vector3<T> &db, Vector3<T> &dx),
            {
              // dx = pa * db + da * pb;
              dx.setZero();
              for (size_t row = 0; row < 3; row++) {
                for (size_t col = 0; col < 3; col++) {
                  dx[row] += da(row, col) * pb[col];
                  dx[row] += pa(row, col) * db[col];
                }
              }
            })
TRACTOR_D_T(reverse, fg_mat3_vec3, mul,
            (const Matrix3<T> &pa, const Vector3<T> &pb, const Matrix3<T> &px,
             Matrix3<T> &da, Vector3<T> &db, const Vector3<T> &dx),
            {
              da.setZero();
              db.setZero();
              for (size_t row = 0; row < 3; row++) {
                for (size_t col = 0; col < 3; col++) {
                  da(row, col) += dx[row] * pb[col];
                  db[col] += dx[row] * pa(row, col);
                }
              }
            })

TRACTOR_OP_T(fg_mat3, add, (const Matrix3<T> &a, const Matrix3<T> &b),
             { return a + b; })
TRACTOR_D_T(prepare, fg_mat3, add,
            (const Matrix3<T> &a, const Matrix3<T> &b, const Matrix3<T> &x), {})
TRACTOR_D_T(forward, fg_mat3, add,
            (const Matrix3<T> &da, const Matrix3<T> &db, Matrix3<T> &dx),
            { dx = da + db; })
TRACTOR_D_T(reverse, fg_mat3, add,
            (Matrix3<T> & da, Matrix3<T> &db, const Matrix3<T> &dx), {
              da = dx;
              db = dx;
            })

TRACTOR_OP_T(fg_mat3, zero, (Matrix3<T> & x), { x.setZero(); })
TRACTOR_D_T(prepare, fg_mat3, zero, (const Matrix3<T> &x), {})
TRACTOR_D_T(forward, fg_mat3, zero, (Matrix3<T> & dx), { dx.setZero(); })
TRACTOR_D_T(reverse, fg_mat3, zero, (const Matrix3<T> &dx), {})

// ---------------------------------------------------------

TRACTOR_OP_T(fg_vec3, minus, (const Vector3<T> &a), { return -a; })
TRACTOR_D_T(prepare, fg_vec3, minus, (const Vector3<T> &a, const Vector3<T> &x),
            {})
TRACTOR_D_T(forward, fg_vec3, minus, (const Vector3<T> &da, Vector3<T> &dx),
            { dx = -da; })
TRACTOR_D_T(reverse, fg_vec3, minus, (Vector3<T> & da, const Vector3<T> &dx),
            { da = -dx; })

TRACTOR_OP_T(fg_vec3, zero, (Vector3<T> & x), { x.setZero(); })
TRACTOR_D_T(prepare, fg_vec3, zero, (const Vector3<T> &x), {})
TRACTOR_D_T(forward, fg_vec3, zero, (Vector3<T> & dx), { dx.setZero(); })
TRACTOR_D_T(reverse, fg_vec3, zero, (const Vector3<T> &dx), {})

TRACTOR_OP_T(fg_vec3, move, (const Vector3<T> &v), { return Vector3<T>(v); })
TRACTOR_D_T(prepare, fg_vec3, move, (const Vector3<T> &a, const Vector3<T> &x),
            {})
TRACTOR_D_T(forward, fg_vec3, move, (const Vector3<T> &da, Vector3<T> &dx),
            { dx = da; })
TRACTOR_D_T(reverse, fg_vec3, move, (Vector3<T> & da, const Vector3<T> &dx),
            { da = dx; })

TRACTOR_OP_T(fg_vec3, add, (const Vector3<T> &a, const Vector3<T> &b),
             { return a + b; })
TRACTOR_D_T(prepare, fg_vec3, add,
            (const Vector3<T> &a, const Vector3<T> &b, const Vector3<T> &x), {})
TRACTOR_D_T(forward, fg_vec3, add,
            (const Vector3<T> &da, const Vector3<T> &db, Vector3<T> &dx),
            { dx = da + db; })
TRACTOR_D_T(reverse, fg_vec3, add,
            (Vector3<T> & da, Vector3<T> &db, const Vector3<T> &dx), {
              da = dx;
              db = dx;
            })

TRACTOR_OP_T(fg_vec3, sub, (const Vector3<T> &a, const Vector3<T> &b),
             { return a - b; })
TRACTOR_D_T(prepare, fg_vec3, sub,
            (const Vector3<T> &a, const Vector3<T> &b, const Vector3<T> &x), {})
TRACTOR_D_T(forward, fg_vec3, sub,
            (const Vector3<T> &da, const Vector3<T> &db, Vector3<T> &dx),
            { dx = da - db; })
TRACTOR_D_T(reverse, fg_vec3, sub,
            (Vector3<T> & da, Vector3<T> &db, const Vector3<T> &dx), {
              da = dx;
              db = -dx;
            })

TRACTOR_OP_T(fg_vec3_s, mul, (const Vector3<T> &a, const T &b),
             { return a * b; })
TRACTOR_D_T(prepare, fg_vec3_s, mul,
            (const Vector3<T> &a, const T &b, const Vector3<T> &x,
             Vector3<T> &va, T &vb),
            {
              va = a;
              vb = b;
            })
TRACTOR_D_T(forward, fg_vec3_s, mul,
            (const Vector3<T> &va, const T &vb, const Vector3<T> &da,
             const T &db, Vector3<T> &dx),
            {
              dx = da * vb + va * db;
              /*dx.x() = da.x() * vb + db * va.x();
              dx.x() = da.y() * vb + db * va.y();
              dx.x() = da.z() * vb + db * va.z();*/
            })
TRACTOR_D_T(reverse, fg_vec3_s, mul,
            (const Vector3<T> &va, const T &vb, Vector3<T> &da, T &db,
             const Vector3<T> &dx),
            {
              da = dx * vb;
              db = dx.x() * va.x() + dx.y() * va.y() + dx.z() * va.z();
            })

TRACTOR_OP_T(fg_s_vec3, mul, (const T &a, const Vector3<T> &b),
             { return a * b; })
TRACTOR_D_T(prepare, fg_s_vec3, mul,
            (const T &a, const Vector3<T> &b, const Vector3<T> &x, T &va,
             Vector3<T> &vb),
            {
              va = a;
              vb = b;
            })
TRACTOR_D_T(forward, fg_s_vec3, mul,
            (const T &va, const Vector3<T> &vb, const T &da,
             const Vector3<T> &db, Vector3<T> &dx),
            { dx = da * vb + va * db; })
TRACTOR_D_T(reverse, fg_s_vec3, mul,
            (const T &va, const Vector3<T> &vb, T &da, Vector3<T> &db,
             const Vector3<T> &dx),
            {
              da = dx.x() * vb.x() + dx.y() * vb.y() + dx.z() * vb.z();
              db = dx * va;
            })

//    | da.x da.y da.z db.x db.y db.z
// ---+------------------------------
// dx | vb.x vb.y vb.z va.x va.y va.z
TRACTOR_OP_T(fg_vec3, dot, (const Vector3<T> &a, const Vector3<T> &b),
             { return dot(a, b); })
TRACTOR_D_T(prepare, fg_vec3, dot,
            (const Vector3<T> &a, const Vector3<T> &b, const T &x,
             Vector3<T> &va, Vector3<T> &vb),
            {
              va = a;
              vb = b;
            })
TRACTOR_D_T(forward, fg_vec3, dot,
            (const Vector3<T> &va, const Vector3<T> &vb, const Vector3<T> &da,
             const Vector3<T> &db, T &dx),
            {
              // dx = (va.x() * db.x() + va.y() * db.y() + va.z() * db.z()) +
              //       (da.x() * vb.x() + da.y() * vb.y() + da.z() * vb.z());
              dx = dot(va, db) + dot(da, vb);
            })
TRACTOR_D_T(reverse, fg_vec3, dot,
            (const Vector3<T> &va, const Vector3<T> &vb, Vector3<T> &da,
             Vector3<T> &db, const T &dx),
            {
              // da.x() = dx * vb.x();
              // da.y() = dx * vb.y();
              // da.z() = dx * vb.z();
              // db.x() = dx * va.x();
              // db.y() = dx * va.y();
              // db.z() = dx * va.z();
              da = vb * dx;
              db = va * dx;
            })

//      |  da.x  da.y   da.z  |  db.x  db.y   db.z
// -----+---------------------+------------------
// dx.x |   0   +vb.z  -vb.y  |   0   -va.z  +va.y
// dx.y | -vb.z   0     vb.x  |  va.z   0    -va.x
// dx.z |  vb.y -vb.x    0    | -va.y  va.x   0
TRACTOR_OP_T(fg_vec3, cross, (const Vector3<T> &a, const Vector3<T> &b),
             { return cross(a, b); })
TRACTOR_D_T(prepare, fg_vec3, cross,
            (const Vector3<T> &a, const Vector3<T> &b, const Vector3<T> &x,
             Vector3<T> &va, Vector3<T> &vb),
            {
              va = a;
              vb = b;
            })
TRACTOR_D_T(forward, fg_vec3, cross,
            (const Vector3<T> &va, const Vector3<T> &vb, const Vector3<T> &da,
             const Vector3<T> &db, Vector3<T> &dx),
            {
              dx.x() = (da.y() * vb.z() - da.z() * vb.y()) +
                       (va.y() * db.z() - va.z() * db.y());
              dx.y() = (da.z() * vb.x() - da.x() * vb.z()) +
                       (va.z() * db.x() - va.x() * db.z());
              dx.z() = (da.x() * vb.y() - da.y() * vb.x()) +
                       (va.x() * db.y() - va.y() * db.x());
            })
TRACTOR_D_T(reverse, fg_vec3, cross,
            (const Vector3<T> &va, const Vector3<T> &vb, Vector3<T> &da,
             Vector3<T> &db, const Vector3<T> &dx),
            {
              da.x() = vb.y() * dx.z() - vb.z() * dx.y();
              da.y() = vb.z() * dx.x() - vb.x() * dx.z();
              da.z() = vb.x() * dx.y() - vb.y() * dx.x();
              db.x() = va.z() * dx.y() - va.y() * dx.z();
              db.y() = va.x() * dx.z() - va.z() * dx.x();
              db.z() = va.y() * dx.x() - va.x() * dx.y();
            })

TRACTOR_OP(fg_vec3_unpack, (const Vector3<T> &v, T &x, T &y, T &z),
           { fg_vec3_unpack(v, x, y, z); })
TRACTOR_D(prepare, fg_vec3_unpack,
          (const Vector3<T> &v, const T &x, const T &y, const T &z), {})
TRACTOR_D(forward, fg_vec3_unpack, (const Vector3<T> &dv, T &dx, T &dy, T &dz),
          {
            dx = dv.x();
            dy = dv.y();
            dz = dv.z();
          })
TRACTOR_D(reverse, fg_vec3_unpack,
          (Vector3<T> & dv, const T &dx, const T &dy, const T &dz), {
            dv.x() = dx;
            dv.y() = dy;
            dv.z() = dz;
          })

TRACTOR_OP(fg_vec3_pack, (const T &x, const T &y, const T &z, Vector3<T> &vec),
           { fg_vec3_pack(x, y, z, vec); })
TRACTOR_D(prepare, fg_vec3_pack,
          (const T &x, const T &y, const T &z, const Vector3<T> &v), {})
TRACTOR_D(forward, fg_vec3_pack,
          (const T &x, const T &y, const T &z, Vector3<T> &v), {
            v.x() = x;
            v.y() = y;
            v.z() = z;
          })
TRACTOR_D(reverse, fg_vec3_pack, (T & x, T &y, T &z, const Vector3<T> &v), {
  x = v.x();
  y = v.y();
  z = v.z();
})

/*
template <class Scalar> void goal(const Var<Vector3<Scalar>> &v) {
  Var<Scalar> x, y, z;
  fg_vec3_unpack(v, x, y, z);
  goal(x);
  goal(y);
  goal(z);
}
*/

TRACTOR_OP_T(fg_quat, zero, (Quaternion<T> & x), { x.setZero(); })
TRACTOR_D_T(prepare, fg_quat, zero, (const Quaternion<T> &x), {})
TRACTOR_D_T(forward, fg_quat, zero, (Quaternion<T> & dx), { dx.setZero(); })
TRACTOR_D_T(reverse, fg_quat, zero, (const Quaternion<T> &dx), {})

TRACTOR_OP_T(fg_quat, move, (const Quaternion<T> &v),
             { return Quaternion<T>(v); })
TRACTOR_D_T(prepare, fg_quat, move,
            (const Quaternion<T> &a, const Quaternion<T> &x), {})
TRACTOR_D_T(forward, fg_quat, move, (const Vector3<T> &da, Vector3<T> &dx),
            { dx = da; })
TRACTOR_D_T(reverse, fg_quat, move, (Vector3<T> & da, const Vector3<T> &dx),
            { da = dx; })

TRACTOR_GRADIENT_TYPE_TEMPLATE(Quaternion<T>, Vector3<T>);

//         da.x   da.y   da.z
// -----+-----------------
// dx.x |   0     vb.z  -vb.y
// dx.y | -vb.z    0     vb.x
// dx.z |  vb.y  -vb.x    0
TRACTOR_OP_T(fg_quat_vec3, mul, (const Quaternion<T> &a, const Vector3<T> &b),
             { return a * b; })
TRACTOR_D_T(prepare, fg_quat_vec3, mul,
            (const Quaternion<T> &a, const Vector3<T> &b, const Vector3<T> &x,
             Quaternion<T> &va, Vector3<T> &vb),
            {
              va = a;
              vb = b;
            })
TRACTOR_D_T(forward, fg_quat_vec3, mul,
            (const Quaternion<T> &va, const Vector3<T> &vb,
             const Vector3<T> &da, const Vector3<T> &db, Vector3<T> &dx),
            {
              // dx = Quaternion<T>(da.x(), da.y(), da.z(), 0.0) * vb + va * db;
              // dx = va * db + cross(da, vb);
              dx = va * db + cross(da, va * vb);
              /*
              return Vector3<T>(a.y() * b.z() - a.z() * b.y(), //
                              a.z() * b.x() - a.x() * b.z(), //
                              a.x() * b.y() - a.y() * b.x()  //
                              */
            })
TRACTOR_D_T(reverse, fg_quat_vec3, mul,
            (const Quaternion<T> &va, const Vector3<T> &vb, Vector3<T> &da,
             Vector3<T> &db, const Vector3<T> &dx),
            {
              da = cross(va * vb, dx);
              db = va.inverse() * dx;
            })

/*
TRACTOR_OP_T(fg_quat, mul, (const Quaternion<T> &a, const Quaternion<T> &b),
             { return a * b; })
TRACTOR_D_T(prepare, fg_quat, mul,
            (const Quaternion<T> &a, const Quaternion<T> &b, const Quaternion<T>
&x, Quaternion<T> &va, Quaternion<T> &vb),
            {
              va = a;
              vb = b;
            })
TRACTOR_D_T(forward, fg_quat, mul,
            (const Quaternion<T> &va, const Quaternion<T> &vb, const Vector3<T>
&da, const Vector3<T> &db, Vector3<T> &dx),
            {
              // dx = (va * Quaternion<T>(db.x(), db.y(), db.z(), 1.0)).vec() +
              //       (Quaternion<T>(da.x(), da.y(), da.z(), 1.0) * vb).vec();
              dx = va * db + da;
            })
TRACTOR_D_T(reverse, fg_quat, mul,
            (const Quaternion<T> &va, const Quaternion<T> &vb, Vector3<T> &da,
Vector3<T> &db, const Vector3<T> &dx),
            {
              // da = (Quaternion<T>(dx.x(), dx.y(), dx.z(), 1.0) *
              // vb.inverse()).vec(); db = (va.inverse() * Quaternion<T>(dx.x(),
              // dx.y(), dx.z(), 1.0)).vec();
              da = dx;
              db = va.inverse() * dx;
            })
*/

TRACTOR_OP_T(fg_quat, mul, (const Quaternion<T> &a, const Quaternion<T> &b),
             { return a * b; })
TRACTOR_D_T(prepare, fg_quat, mul,
            (const Quaternion<T> &a, const Quaternion<T> &b,
             const Quaternion<T> &x, Quaternion<T> &va),
            { va = a; })
TRACTOR_D_T(forward, fg_quat, mul,
            (const Quaternion<T> &va, const Vector3<T> &da,
             const Vector3<T> &db, Vector3<T> &dx),
            { dx = va * db + da; })
TRACTOR_D_T(reverse, fg_quat, mul,
            (const Quaternion<T> &va, Vector3<T> &da, Vector3<T> &db,
             const Vector3<T> &dx),
            {
              da = dx;
              db = va.inverse() * dx;
            })

template <class T> auto fg_quat_inverse(const Quaternion<T> &a) {
  return a.inverse();
}
TRACTOR_OP(fg_quat_inverse, (const Quaternion<T> &a),
           { return fg_quat_inverse(a); })
TRACTOR_D(prepare, fg_quat_inverse,
          (const Quaternion<T> &a, const Quaternion<T> &x), {})
TRACTOR_D(forward, fg_quat_inverse, (const Vector3<T> &a, Vector3<T> &x),
          { x = -a; })
TRACTOR_D(reverse, fg_quat_inverse, (Vector3<T> & a, const Vector3<T> &x),
          { a = -x; })

TRACTOR_OP(fg_quat_unpack, (const Quaternion<T> &q, T &x, T &y, T &z, T &w),
           { fg_quat_unpack(q, x, y, z, w); })

TRACTOR_OP(fg_quat_pack,
           (const T &x, const T &y, const T &z, const T &w, Quaternion<T> &vec),
           { fg_quat_pack(x, y, z, w, vec); })

// TRACTOR_OP(fg_quat_residual, (const Quaternion<T> &a), { return a.vec() *
// T(2);
// })
TRACTOR_OP(fg_quat_residual, (const Quaternion<T> &a),
           { return fg_quat_residual(a); })
TRACTOR_D(prepare, fg_quat_residual,
          (const Quaternion<T> &a, const Vector3<T> &x), {})
TRACTOR_D(forward, fg_quat_residual, (const Vector3<T> &a, Vector3<T> &x),
          { x = a; })
TRACTOR_D(reverse, fg_quat_residual, (Vector3<T> & a, const Vector3<T> &x),
          { a = x; })

TRACTOR_OP(fg_angle_axis_quat, (const T &angle, const Vector3<T> &axis),
           { return fg_angle_axis_quat(angle, axis); })
TRACTOR_D(prepare, fg_angle_axis_quat,
          (const T &angle, const Vector3<T> &axis, const Quaternion<T> &rot,
           T &v_angle, Vector3<T> &v_axis),
          {
            v_angle = angle;
            v_axis = axis;
          })
TRACTOR_D(forward, fg_angle_axis_quat,
          (const T &v_angle, const Vector3<T> &v_axis, const T &d_angle,
           const Vector3<T> &d_axis, Vector3<T> &d_rot),
          { d_rot = v_axis * d_angle + d_axis * v_angle; })
TRACTOR_D(reverse, fg_angle_axis_quat,
          (const T &v_angle, const Vector3<T> &v_axis, T &d_angle,
           Vector3<T> &d_axis, const Vector3<T> &d_rot),
          {
            // d_angle = d_rot.x() * v_axis.x() + d_rot.y() * v_axis.y() +
            //          d_rot.z() * v_axis.z();
            d_angle = dot(d_rot, v_axis);
            d_axis = d_rot * v_angle;
          })

TRACTOR_GRADIENT_TYPE_TEMPLATE(Pose<T>, Twist<T>);

TRACTOR_OP_T(fg_pose, zero, (Pose<T> & x), { x.setZero(); })
TRACTOR_D_T(prepare, fg_pose, zero, (const Pose<T> &x), {})
TRACTOR_D_T(forward, fg_pose, zero, (Pose<T> & dx), { dx.setZero(); })
TRACTOR_D_T(reverse, fg_pose, zero, (const Pose<T> &dx), {})

TRACTOR_OP_T(fg_twist, zero, (Twist<T> & x), { x.setZero(); })
TRACTOR_D_T(prepare, fg_twist, zero, (const Twist<T> &x), {})
TRACTOR_D_T(forward, fg_twist, zero, (Twist<T> & dx), { dx.setZero(); })
TRACTOR_D_T(reverse, fg_twist, zero, (const Twist<T> &dx), {})

TRACTOR_OP_T(fg_pose, move, (const Pose<T> &v), { return Pose<T>(v); })
TRACTOR_D_T(prepare, fg_pose, move, (const Pose<T> &a, const Pose<T> &x), {})
TRACTOR_D_T(forward, fg_pose, move, (const Twist<T> &da, Twist<T> &dx),
            { dx = da; })
TRACTOR_D_T(reverse, fg_pose, move, (Twist<T> & da, const Twist<T> &dx),
            { da = dx; })

TRACTOR_OP_T(fg_twist, move, (const Twist<T> &v), { return Twist<T>(v); })
TRACTOR_D_T(prepare, fg_twist, move, (const Twist<T> &a, const Twist<T> &x), {})
TRACTOR_D_T(forward, fg_twist, move, (const Twist<T> &da, Twist<T> &dx),
            { dx = da; })
TRACTOR_D_T(reverse, fg_twist, move, (Twist<T> & da, const Twist<T> &dx),
            { da = dx; })

TRACTOR_OP_T(fg_twist, add, (const Twist<T> &a, const Twist<T> &b),
             { return a + b; })
TRACTOR_D_T(prepare, fg_twist, add,
            (const Twist<T> &a, const Twist<T> &b, const Twist<T> &x), {})
TRACTOR_D_T(forward, fg_twist, add,
            (const Twist<T> &da, const Twist<T> &db, Twist<T> &dx),
            { dx = da + db; })
TRACTOR_D_T(reverse, fg_twist, add,
            (Twist<T> & da, Twist<T> &db, const Twist<T> &dx), {
              da = dx;
              db = dx;
            })

TRACTOR_OP_T(fg_twist_s, mul, (const Twist<T> &a, const T &b),
             { return a * b; })
TRACTOR_D_T(prepare, fg_twist_s, mul,
            (const Twist<T> &a, const T &b, const Twist<T> &x, Twist<T> &va,
             T &vb),
            {
              va = a;
              vb = b;
            })
TRACTOR_D_T(forward, fg_twist_s, mul,
            (const Twist<T> &va, const T &vb, const Twist<T> &da, const T &db,
             Twist<T> &dx),
            { dx = da * vb + va * db; })
TRACTOR_D_T(reverse, fg_twist_s, mul,
            (const Twist<T> &va, const T &vb, Twist<T> &da, T &db,
             const Twist<T> &dx),
            {
              da = dx * vb;
              db = dot(dx.translation(), va.translation()) +
                   dot(dx.rotation(), va.rotation());
            })

TRACTOR_OP_T(fg_twist, sub, (const Twist<T> &a, const Twist<T> &b),
             { return a - b; })
TRACTOR_D_T(prepare, fg_twist, sub,
            (const Twist<T> &a, const Twist<T> &b, const Twist<T> &x), {})
TRACTOR_D_T(forward, fg_twist, sub,
            (const Twist<T> &da, const Twist<T> &db, Twist<T> &dx),
            { dx = da - db; })
TRACTOR_D_T(reverse, fg_twist, sub,
            (Twist<T> & da, Twist<T> &db, const Twist<T> &dx), {
              da = dx;
              db.translation() = -dx.translation();
              db.rotation() = -dx.rotation();
            })

template <class T> struct PoseMulState {
  Quaternion<T> ar;
  Vector3<T> arbt;
  Quaternion<T> arinv;
};
TRACTOR_OP_T(fg_pose, mul, (const Pose<T> &a, const Pose<T> &b),
             { return a * b; })
TRACTOR_D_T(prepare, fg_pose, mul,
            (const Pose<T> &a, const Pose<T> &b, const Pose<T> &x,
             PoseMulState<T> &v),
            {
              // v.at = at;
              v.ar = a.orientation();
              // v.bt = bt;
              // v.br = br;
              v.arbt = a.orientation() * b.translation();
              v.arinv = a.orientation().inverse();
            })
TRACTOR_D_T(forward, fg_pose, mul,
            (const PoseMulState<T> &v, const Twist<T> &da, const Twist<T> &db,
             Twist<T> &dx),
            {

              // xt = at + ar * bt
              // dxt = dat + v.ar * dbt + cross(dar, v.ar * v.bt);
              dx.translation() = da.translation() + v.ar * db.translation() +
                                 cross(da.rotation(), v.arbt);

              // xr = ar * br
              dx.rotation() = v.ar * db.rotation() + da.rotation();
            })
TRACTOR_D_T(reverse, fg_pose, mul,
            (const PoseMulState<T> &v, Twist<T> &da, Twist<T> &db,
             const Twist<T> &dx),
            {

              // xt = at + ar * bt

              // dat = dxt;
              // dar = cross(v.ar * v.bt, dxt);
              // dbt = v.ar.inverse() * dxt;

              da.translation() = dx.translation();
              da.rotation() = cross(v.arbt, dx.translation());
              db.translation() = v.arinv * dx.translation();

              // xr = ar * br

              // dar = dar + dxr;
              // dbr = v.ar.inverse() * dxr;

              da.rotation() = da.rotation() + dx.rotation();
              db.rotation() = v.arinv * dx.rotation();

            })

template <class T> struct PoseVec3MulState {
  Quaternion<T> ar;
  Vector3<T> arbt;
  Quaternion<T> arinv;
};
TRACTOR_OP_T(fg_pose_vec3, mul, (const Pose<T> &a, const Vector3<T> &b),
             { return a * b; })
TRACTOR_D_T(prepare, fg_pose_vec3, mul,
            (const Pose<T> &a, const Vector3<T> &b, const Vector3<T> &x,
             PoseVec3MulState<T> &v),
            {
              v.ar = a.orientation();
              v.arbt = a.orientation() * b;
              v.arinv = a.orientation().inverse();
            })
TRACTOR_D_T(forward, fg_pose_vec3, mul,
            (const PoseVec3MulState<T> &v, const Twist<T> &da,
             const Vector3<T> &db, Vector3<T> &dx),
            {
              dx = da.translation() + v.ar * db + cross(da.rotation(), v.arbt);
            })
TRACTOR_D_T(reverse, fg_pose_vec3, mul,
            (const PoseVec3MulState<T> &v, Twist<T> &da, Vector3<T> &db,
             const Vector3<T> &dx),
            {
              da.translation() = dx;
              da.rotation() = cross(v.arbt, dx);
              db = v.arinv * dx;
            })

template <class T> struct FGAngleAxisPoseState {
  T angle;
  Vector3<T> axis;
};
TRACTOR_OP(fg_angle_axis_pose, (const T &angle, const Vector3<T> &axis),
           { return fg_angle_axis_pose(angle, axis); })
TRACTOR_D(prepare, fg_angle_axis_pose,
          (const T &angle, const Vector3<T> &axis, const Pose<T> &pose,
           FGAngleAxisPoseState<T> &v),
          {
            v.angle = angle;
            v.axis = axis;
          })
TRACTOR_D(forward, fg_angle_axis_pose,
          (const FGAngleAxisPoseState<T> &v, const T &d_angle,
           const Vector3<T> &d_axis, Twist<T> &d_pose),
          {
            d_pose.translation().setZero();
            d_pose.rotation() = v.axis * d_angle + d_axis * v.angle;
          })
TRACTOR_D(reverse, fg_angle_axis_pose,
          (const FGAngleAxisPoseState<T> &v, T &d_angle, Vector3<T> &d_axis,
           const Twist<T> &d_pose),
          {
            d_angle = dot(d_pose.rotation(), v.axis);
            d_axis = d_pose.rotation() * v.angle;
          })

template <class T> struct PoseAngleAxisPoseState {
  // Pose<T> parent;
  Quaternion<T> parent_orientation;
  Quaternion<T> parent_orientation_inverse;
  Vector3<T> parent_orientation_axis;
  T angle;
  Vector3<T> axis;
};
TRACTOR_OP(fg_pose_angle_axis_pose,
           (const Pose<T> &parent, const T &angle, const Vector3<T> &axis),
           { return fg_pose_angle_axis_pose(parent, angle, axis); })
TRACTOR_D(prepare, fg_pose_angle_axis_pose,
          (const Pose<T> &parent, const T &angle, const Vector3<T> &axis,
           const Pose<T> &pose, PoseAngleAxisPoseState<T> &v),
          {
            // v.parent = parent;
            v.parent_orientation = parent.orientation();
            v.parent_orientation_inverse = parent.orientation().inverse();
            v.parent_orientation_axis = parent.orientation() * axis;
            v.angle = angle;
            v.axis = axis;
          })
TRACTOR_D(forward, fg_pose_angle_axis_pose,
          (const PoseAngleAxisPoseState<T> &v, const Twist<T> &d_parent,
           const T &d_angle, const Vector3<T> &d_axis, Twist<T> &d_pose),
          {

            // d_pose.translation() = d_parent.translation();
            // d_pose.rotation() = d_parent.rotation() +
            //                    (v.parent.orientation() * v.axis) * d_angle +
            //                    (v.parent.orientation() * d_axis) * v.angle;

            // d_pose.translation() = d_parent.translation();
            // d_pose.rotation() = d_parent.rotation() +
            //                    (v.parent_orientation_axis) * d_angle +
            //                    (v.parent_orientation * d_axis) * v.angle;

            // dx.translation() = da.translation() + v.ar * db.translation() +
            //                   cross(da.rotation(), v.arbt);
            // dx.rotation() = v.ar * db.rotation() + da.rotation();

            d_pose.translation() = d_parent.translation();
            d_pose.rotation() = d_parent.rotation() +
                                (v.parent_orientation_axis) * d_angle +
                                (v.parent_orientation * d_axis) * v.angle;
          })
TRACTOR_D(reverse, fg_pose_angle_axis_pose,
          (const PoseAngleAxisPoseState<T> &v, Twist<T> &d_parent, T &d_angle,
           Vector3<T> &d_axis, const Twist<T> &d_pose),
          {

            // d_parent.translation() = d_pose.translation();
            // d_parent.rotation() = d_pose.rotation();
            // d_angle = dot(v.parent.orientation().inverse() *
            // d_pose.rotation(),
            //              v.axis);
            // d_axis = (v.parent.orientation().inverse() * d_pose.rotation()) *
            //         v.angle;

            d_parent.translation() = d_pose.translation();
            d_parent.rotation() = d_pose.rotation();
            d_angle =
                dot(v.parent_orientation_inverse * d_pose.rotation(), v.axis);
            d_axis =
                (v.parent_orientation_inverse * d_pose.rotation()) * v.angle;
          })

TRACTOR_OP(fg_pose_translation, (const Pose<T> &pose),
           { return fg_pose_translation(pose); })
TRACTOR_D(prepare, fg_pose_translation,
          (const Pose<T> &pose, const Vector3<T> &vec), {})
TRACTOR_D(forward, fg_pose_translation,
          (const Twist<T> &twist, Vector3<T> &translation),
          { translation = twist.translation(); })
TRACTOR_D(reverse, fg_pose_translation,
          (Twist<T> & twist, const Vector3<T> &translation), {
            twist.translation() = translation;
            twist.rotation().setZero();
          })

TRACTOR_OP(fg_pose_orientation, (const Pose<T> &pose),
           { return fg_pose_orientation(pose); })
TRACTOR_D(prepare, fg_pose_orientation,
          (const Pose<T> &pose, const Quaternion<T> &orientation), {})
TRACTOR_D(forward, fg_pose_orientation,
          (const Twist<T> &twist, Vector3<T> &rotation),
          { rotation = twist.rotation(); })
TRACTOR_D(reverse, fg_pose_orientation,
          (Twist<T> & twist, const Vector3<T> &rotation), {
            twist.rotation() = rotation;
            twist.translation().setZero();
          })

TRACTOR_OP(fg_translation_pose, (const Vector3<T> &translation),
           { return fg_translation_pose(translation); })
TRACTOR_D(prepare, fg_translation_pose,
          (const Vector3<T> &translation, const Pose<T> &pose), {})
TRACTOR_D(forward, fg_translation_pose,
          (const Vector3<T> &translation, Twist<T> &twist), {
            twist.translation() = translation;
            twist.rotation().setZero();
          })
TRACTOR_D(reverse, fg_translation_pose,
          (Vector3<T> & translation, const Twist<T> &twist),
          { translation = twist.translation(); })

TRACTOR_OP(fg_translation_twist, (const Vector3<T> &translation),
           { return fg_translation_twist(translation); })
TRACTOR_D(prepare, fg_translation_twist,
          (const Vector3<T> &translation, const Twist<T> &pose), {})
TRACTOR_D(forward, fg_translation_twist,
          (const Vector3<T> &translation, Twist<T> &twist), {
            twist.translation() = translation;
            twist.rotation().setZero();
          })
TRACTOR_D(reverse, fg_translation_twist,
          (Vector3<T> & translation, const Twist<T> &twist),
          { translation = twist.translation(); })

TRACTOR_OP(fg_make_twist,
           (const Vector3<T> &translation, const Vector3<T> &rotation),
           { return fg_make_twist(translation, rotation); })
TRACTOR_D(prepare, fg_make_twist,
          (const Vector3<T> &translation, const Vector3<T> &rotation,
           const Twist<T> &twist),
          {})
TRACTOR_D(forward, fg_make_twist,
          (const Vector3<T> &translation, const Vector3<T> &rotation,
           Twist<T> &twist),
          {
            twist.translation() = translation;
            twist.rotation() = rotation;
          })
TRACTOR_D(reverse, fg_make_twist,
          (Vector3<T> & translation, Vector3<T> &rotation,
           const Twist<T> &twist),
          {
            translation = twist.translation();
            rotation = twist.rotation();
          })

TRACTOR_OP(fg_orientation_pose, (const Quaternion<T> &orientation),
           { return fg_orientation_pose(orientation); })
TRACTOR_D(prepare, fg_orientation_pose,
          (const Quaternion<T> &orientation, const Pose<T> &pose), {})
TRACTOR_D(forward, fg_orientation_pose,
          (const Vector3<T> &rotation, Twist<T> &twist), {
            twist.rotation() = rotation;
            twist.translation().setZero();
          })
TRACTOR_D(reverse, fg_orientation_pose,
          (Vector3<T> & rotation, const Twist<T> &twist),
          { rotation = twist.rotation(); })

TRACTOR_OP(fg_pose_translate,
           (const Pose<T> &parent, const Vector3<T> &translation),
           { return fg_pose_translate(parent, translation); })
TRACTOR_D(prepare, fg_pose_translate,
          (const Pose<T> &parent, const Vector3<T> &translation,
           const Pose<T> &pose, Quaternion<T> &parent_orientation),
          { parent_orientation = parent.orientation(); })
TRACTOR_D(forward, fg_pose_translate,
          (const Quaternion<T> &parent_orientation, const Twist<T> &parent,
           const Vector3<T> &translation, Twist<T> &twist),
          {
            twist.rotation() = parent.rotation();
            twist.translation() =
                parent.translation() + parent_orientation * translation;
          })
TRACTOR_D(reverse, fg_pose_translate,
          (const Quaternion<T> &parent_orientation, Twist<T> &parent,
           Vector3<T> &translation, const Twist<T> &twist),
          {
            parent = twist;
            translation = parent_orientation.inverse() * twist.translation();
          })

template <class T> struct PoseResidualState {
  Pose<T> a;
  Pose<T> b;
};
TRACTOR_OP(fg_pose_residual, (const Pose<T> &a, const Pose<T> &b),
           { return fg_pose_residual(a, b); })
TRACTOR_D(prepare, fg_pose_residual,
          (const Pose<T> &a, const Pose<T> &b, const Twist<T> &x,
           PoseResidualState<T> &v),
          {
            v.a = a;
            v.b = b;
          })
TRACTOR_D(forward, fg_pose_residual,
          (const PoseResidualState<T> &v, const Twist<T> &da,
           const Twist<T> &db, Twist<T> &dx),
          {
            // dx.translation() = da.translation() - db.translation();
            // dx.rotation() =
            //    v.a.orientation().inverse() * db.rotation() - da.rotation();
            dx = db - da;
          })
TRACTOR_D(reverse, fg_pose_residual,
          (const PoseResidualState<T> &v, Twist<T> &da, Twist<T> &db,
           const Twist<T> &dx),
          {
            // da.translation() = dx.translation();
            // db.translation() = -dx.translation();
            // da.rotation() = -dx.rotation();
            // db.rotation() = v.a.orientation() * dx.rotation();
            da = -dx;
            db = dx;
          })

TRACTOR_OP(fg_twist_unpack,
           (const Twist<T> &v, T &tx, T &ty, T &tz, T &rx, T &ry, T &rz), {
             tx = v.translation().x();
             ty = v.translation().y();
             tz = v.translation().z();
             rx = v.rotation().x();
             ry = v.rotation().y();
             rz = v.rotation().z();
           })
TRACTOR_D(prepare, fg_twist_unpack,
          (const Twist<T> &v, const T &tx, const T &ty, const T &tz,
           const T &rx, const T &ry, const T &rz),
          {})
TRACTOR_D(forward, fg_twist_unpack,
          (const Twist<T> &v, T &tx, T &ty, T &tz, T &rx, T &ry, T &rz), {
            tx = v.translation().x();
            ty = v.translation().y();
            tz = v.translation().z();
            rx = v.rotation().x();
            ry = v.rotation().y();
            rz = v.rotation().z();
          })
TRACTOR_D(reverse, fg_twist_unpack,
          (Twist<T> & v, const T &tx, const T &ty, const T &tz, const T &rx,
           const T &ry, const T &rz),
          {
            v.translation().x() = tx;
            v.translation().y() = ty;
            v.translation().z() = tz;
            v.rotation().x() = rx;
            v.rotation().y() = ry;
            v.rotation().z() = rz;
          })

TRACTOR_OP(fg_twist_translation, (const Twist<T> &twist),
           { return fg_twist_translation(twist); })
TRACTOR_D(prepare, fg_twist_translation,
          (const Twist<T> &twist, const Vector3<T> &translation), {})
TRACTOR_D(forward, fg_twist_translation,
          (const Twist<T> &twist, Vector3<T> &translation),
          { translation = twist.translation(); })
TRACTOR_D(reverse, fg_twist_translation,
          (Twist<T> & twist, const Vector3<T> &translation), {
            twist.translation() = translation;
            twist.rotation().setZero();
          })

TRACTOR_OP(fg_twist_rotation, (const Twist<T> &twist),
           { return fg_twist_rotation(twist); })
TRACTOR_D(prepare, fg_twist_rotation,
          (const Twist<T> &twist, const Vector3<T> &rotation), {})
TRACTOR_D(forward, fg_twist_rotation,
          (const Twist<T> &twist, Vector3<T> &rotation),
          { rotation = twist.rotation(); })
TRACTOR_D(reverse, fg_twist_rotation,
          (Twist<T> & twist, const Vector3<T> &rotation), {
            twist.translation().setZero();
            twist.rotation() = rotation;
          })

/*
template <class Scalar> void goal(const Var<Twist<Scalar>> &v) {
  Var<Scalar> tx, ty, tz, rx, ry, rz;
  fg_twist_unpack(v, tx, ty, tz, rx, ry, rz);
  goal(tx);
  goal(ty);
  goal(tz);
  goal(rx);
  goal(ry);
  goal(rz);
}
*/

/*
template <class T> void parameter(Pose<T> &pose) {
  parameter(pose.translation());
  parameter(pose.orientation());
}
*/

template <class T> Pose<T> operator+(const Pose<T> &a, const Twist<T> &b) {
  Pose<T> ret;
  ret.translation() =
      a.translation() + b.translation() + cross(b.rotation(), a.translation());
  ret.orientation() =
      normalized(normalized(Quaternion<T>(b.rotation().x() * T(0.5),
                                          b.rotation().y() * T(0.5),
                                          b.rotation().z() * T(0.5), T(1.0))) *
                 a.orientation());
  return ret;
}
TRACTOR_OP_T(fg_pose_twist, add, (const Pose<T> &a, const Twist<T> &b), {
  Pose<T> ret = a + b;
  // std::cout << "add pose twist " << ret << std::endl;
  return ret;
})
TRACTOR_D_T(prepare, fg_pose_twist, add,
            (const Pose<T> &a, const Twist<T> &b, const Pose<T> &x), {})
TRACTOR_D_T(forward, fg_pose_twist, add,
            (const Twist<T> &a, const Twist<T> &b, Twist<T> &x), { x = a + b; })
TRACTOR_D_T(reverse, fg_pose_twist, add,
            (Twist<T> & a, Twist<T> &b, const Twist<T> &x), {
              a = x;
              b = x;
            })

template <class T>
Quaternion<T> operator+(const Quaternion<T> &a, const Vector3<T> &b) {
  return normalized(normalized(Quaternion<T>(b.x() * T(0.5), b.y() * T(0.5),
                                             b.z() * T(0.5), T(1.0))) *
                    a);
}
TRACTOR_OP_T(fg_quat_vec3, add, (const Quaternion<T> &a, const Vector3<T> &b),
             { return a + b; })
TRACTOR_D_T(prepare, fg_quat_vec3, add,
            (const Quaternion<T> &a, const Vector3<T> &b,
             const Quaternion<T> &x),
            {})
TRACTOR_D_T(forward, fg_quat_vec3, add,
            (const Vector3<T> &a, const Vector3<T> &b, Vector3<T> &x),
            { x = a + b; })
TRACTOR_D_T(reverse, fg_quat_vec3, add,
            (Vector3<T> & a, Vector3<T> &b, const Vector3<T> &x), {
              a = x;
              b = x;
            })

template <class T> T gate(const T &a, const T &b) { return a; }
TRACTOR_OP(gate, (const T &a, const T &b), { return a; })
TRACTOR_D(prepare, gate, (const T &a, const T &b, const T &x, T &p), { p = b; })
TRACTOR_D(forward, gate, (const T &p, const T &da, const T &db, T &dx),
          { dx = da * p; })
TRACTOR_D(reverse, gate, (const T &p, T &da, T &db, const T &dx),
          { da = dx * p; })

template <class T> Pose<T> gate(const Pose<T> &a, const T &b) { return a; }
TRACTOR_OP_T(fg_pose_gate, gate, (const Pose<T> &a, const T &b), { return a; })
TRACTOR_D_T(prepare, fg_pose_gate, gate,
            (const Pose<T> &a, const T &b, const Pose<T> &x, T &p), { p = b; })
TRACTOR_D_T(forward, fg_pose_gate, gate,
            (const T &p, const Twist<T> &da, const T &db, Twist<T> &dx), {
              dx.translation() = da.translation() * p;
              dx.rotation() = da.rotation() * p;
            })
TRACTOR_D_T(reverse, fg_pose_gate, gate,
            (const T &p, Twist<T> &da, T &db, const Twist<T> &dx), {
              da.translation() = dx.translation() * p;
              da.rotation() = dx.rotation() * p;
              db = T(0);
            })

// -------------------------------------------------------------------------

TRACTOR_OP(pose_trust_region_constraint, (const Pose<T> &a, const T &tr),
           { return T(0); })
TRACTOR_D(prepare, pose_trust_region_constraint,
          (const Pose<T> &a, const T &tr, const T &x), {})
TRACTOR_D(forward, pose_trust_region_constraint,
          (const Twist<T> &a, const T &tr, T &x), { x = T(0); })
TRACTOR_D(reverse, pose_trust_region_constraint,
          (Twist<T> & a, T &tr, const T &x), {
            a.setZero();
            tr = T(0);
          })
TRACTOR_D(project, pose_trust_region_constraint,
          (const Pose<T> &a, const T &tr, const Twist<T> &da, const T &padding,
           Twist<T> &dx),
          {
            dx = da;
            // for (size_t i = 0; i < 6; i++) {
            //  dx[i] = std::max(-tr, std::min(tr, da[i]));
            //}
          })
TRACTOR_D(barrier_init, pose_trust_region_constraint,
          (const Pose<T> &a, const T &tr, const Twist<T> &da, Twist<T> &dx,
           Twist<T> &ddx),
          {
            for (size_t i = 0; i < 6; i++) {
              T p = da[i];
              T lo2 = -tr;
              T hi2 = +tr;
              T u = T(-1) / std::max(T(0), p - lo2);
              T v = T(+1) / std::max(T(0), hi2 - p);
              dx[i] = u + v;
              ddx[i] = (u * u) + (v * v);
            }
          })
TRACTOR_D(barrier_step, pose_trust_region_constraint,
          (const Twist<T> &dda, const Twist<T> &da, Twist<T> &dx), {
            for (size_t i = 0; i < 6; i++) {
              dx[i] = dda[i] * da[i];
            }
          })
TRACTOR_D(barrier_diagonal, pose_trust_region_constraint,
          (const Twist<T> &dda, Twist<T> &ddx), { ddx = dda; })

// -------------------------------------------------------------------------

template <class T>
auto vector3_trust_region_constraint(const Vector3<T> &a, const T &tr) {
  return T(0);
}
TRACTOR_OP(vector3_trust_region_constraint, (const Vector3<T> &a, const T &tr),
           { return T(0); })
TRACTOR_D(prepare, vector3_trust_region_constraint,
          (const Vector3<T> &a, const T &tr, const T &x), {})
TRACTOR_D(forward, vector3_trust_region_constraint,
          (const Vector3<T> &a, const T &tr, T &x), { x = T(0); })
TRACTOR_D(reverse, vector3_trust_region_constraint,
          (Vector3<T> & a, T &tr, const T &x), {
            a.setZero();
            tr = T(0);
          })
TRACTOR_D(project, vector3_trust_region_constraint,
          (const Vector3<T> &a, const T &tr, const Vector3<T> &da,
           const T &padding, Vector3<T> &dx),
          { dx = da; })
TRACTOR_D(barrier_init, vector3_trust_region_constraint,
          (const Vector3<T> &a, const T &tr, const Vector3<T> &da,
           Vector3<T> &dx, Vector3<T> &ddx),
          {
            for (size_t i = 0; i < 3; i++) {
              T p = da[i];
              T lo2 = -tr;
              T hi2 = +tr;
              T u = T(-1) / std::max(T(0), p - lo2);
              T v = T(+1) / std::max(T(0), hi2 - p);
              dx[i] = u + v;
              ddx[i] = (u * u) + (v * v);
            }
          })
TRACTOR_D(barrier_step, vector3_trust_region_constraint,
          (const Vector3<T> &dda, const Twist<T> &da, Vector3<T> &dx), {
            for (size_t i = 0; i < 6; i++) {
              dx[i] = dda[i] * da[i];
            }
          })
TRACTOR_D(barrier_diagonal, vector3_trust_region_constraint,
          (const Vector3<T> &dda, Vector3<T> &ddx), { ddx = dda; })

// -------------------------------------------------------------------------

template <class T>
auto quaternion_trust_region_constraint(const Quaternion<T> &a, const T &tr) {
  return T(0);
}
TRACTOR_OP(quaternion_trust_region_constraint,
           (const Quaternion<T> &a, const T &tr), { return T(0); })
TRACTOR_D(prepare, quaternion_trust_region_constraint,
          (const Quaternion<T> &a, const T &tr, const T &x), {})
TRACTOR_D(forward, quaternion_trust_region_constraint,
          (const Vector3<T> &a, const T &tr, T &x), { x = T(0); })
TRACTOR_D(reverse, quaternion_trust_region_constraint,
          (Vector3<T> & a, T &tr, const T &x), {
            a.setZero();
            tr = T(0);
          })
TRACTOR_D(project, quaternion_trust_region_constraint,
          (const Quaternion<T> &a, const T &tr, const Vector3<T> &da,
           const T &padding, Vector3<T> &dx),
          { dx = da; })
TRACTOR_D(barrier_init, quaternion_trust_region_constraint,
          (const Quaternion<T> &a, const T &tr, const Vector3<T> &da,
           Vector3<T> &dx, Vector3<T> &ddx),
          {
            for (size_t i = 0; i < 6; i++) {
              T p = da[i];
              T lo2 = -tr;
              T hi2 = +tr;
              T u = T(-1) / std::max(T(0), p - lo2);
              T v = T(+1) / std::max(T(0), hi2 - p);
              dx[i] = u + v;
              ddx[i] = (u * u) + (v * v);
            }
          })
TRACTOR_D(barrier_step, quaternion_trust_region_constraint,
          (const Vector3<T> &dda, const Vector3<T> &da, Vector3<T> &dx), {
            for (size_t i = 0; i < 6; i++) {
              dx[i] = dda[i] * da[i];
            }
          })
TRACTOR_D(barrier_diagonal, quaternion_trust_region_constraint,
          (const Vector3<T> &dda, Vector3<T> &ddx), { ddx = dda; })

// -------------------------------------------------------------------------

template <class T> inline auto squaredNorm(const Var<Vector3<T>> &v) {
  return T(dot(v, v));
}

template <class T> inline auto norm(const Var<Vector3<T>> &v) {
  return T(sqrt(dot(v, v)));
}

template <class T> inline auto normalized(const Var<Vector3<T>> &v) {
  return v * (T(1) / norm(v));
}

} // namespace tractor
