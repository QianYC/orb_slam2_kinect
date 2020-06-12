//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_SE3_OPS_H
#define ORB_SLAM2_KINETIC_SE3_OPS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
    using namespace Eigen;

    inline Matrix3d skew(const Vector3d&v);
    inline Vector3d deltaR(const Matrix3d& R);
    inline Vector2d project(const Vector3d&);
    inline Vector3d project(const Vector4d&);
    inline Vector3d unproject(const Vector2d&);
    inline Vector4d unproject(const Vector3d&);

#include "se3_ops.hpp"

}

#endif //ORB_SLAM2_KINETIC_SE3_OPS_H
