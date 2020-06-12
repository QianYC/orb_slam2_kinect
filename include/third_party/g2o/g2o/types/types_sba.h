//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_TYPES_SBA_H
#define ORB_SLAM2_KINETIC_TYPES_SBA_H

#include "../core/base_vertex.h"

#include <Eigen/Geometry>
#include <iostream>

namespace g2o {

/**
 * \brief Point vertex, XYZ
 */
    class VertexSBAPointXYZ : public BaseVertex<3, Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexSBAPointXYZ();
        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        virtual void setToOriginImpl() {
            _estimate.fill(0.);
        }

        virtual void oplusImpl(const double* update)
        {
            Eigen::Map<const Vector3d> v(update);
            _estimate += v;
        }
    };

} // end namespace


#endif //ORB_SLAM2_KINETIC_TYPES_SBA_H
