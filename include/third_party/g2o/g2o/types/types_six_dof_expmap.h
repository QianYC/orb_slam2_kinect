//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_TYPES_SIX_DOF_EXPMAP_H
#define ORB_SLAM2_KINETIC_TYPES_SIX_DOF_EXPMAP_H

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "../core/base_unary_edge.h"
#include "se3_ops.h"
#include "se3quat.h"
#include "types_sba.h"
#include <Eigen/Geometry>

namespace g2o {
    namespace types_six_dof_expmap {
        void init();
    }

    using namespace Eigen;

    typedef Matrix<double, 6, 6> Matrix6d;


/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
    class  VertexSE3Expmap : public BaseVertex<6, SE3Quat>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexSE3Expmap();

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        virtual void setToOriginImpl() {
            _estimate = SE3Quat();
        }

        virtual void oplusImpl(const double* update_)  {
            Eigen::Map<const Vector6d> update(update_);
            setEstimate(SE3Quat::exp(update)*estimate());
        }
    };


    class  EdgeSE3ProjectXYZ: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZ();

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
            Vector2d obs(_measurement);
            _error = obs-cam_project(v1->estimate().map(v2->estimate()));
        }

        bool isDepthPositive() {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
            return (v1->estimate().map(v2->estimate()))(2)>0.0;
        }


        virtual void linearizeOplus();

        Vector2d cam_project(const Vector3d & trans_xyz) const;

        double fx, fy, cx, cy;
    };


    class  EdgeStereoSE3ProjectXYZ: public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeStereoSE3ProjectXYZ();

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
            Vector3d obs(_measurement);
            _error = obs - cam_project(v1->estimate().map(v2->estimate()),bf);
        }

        bool isDepthPositive() {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
            return (v1->estimate().map(v2->estimate()))(2)>0.0;
        }


        virtual void linearizeOplus();

        Vector3d cam_project(const Vector3d & trans_xyz, const float &bf) const;

        double fx, fy, cx, cy, bf;
    };

    class  EdgeSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZOnlyPose(){}

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            Vector2d obs(_measurement);
            _error = obs-cam_project(v1->estimate().map(Xw));
        }

        bool isDepthPositive() {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            return (v1->estimate().map(Xw))(2)>0.0;
        }


        virtual void linearizeOplus();

        Vector2d cam_project(const Vector3d & trans_xyz) const;

        Vector3d Xw;
        double fx, fy, cx, cy;
    };


    class  EdgeStereoSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeStereoSE3ProjectXYZOnlyPose(){}

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            Vector3d obs(_measurement);
            _error = obs - cam_project(v1->estimate().map(Xw));
        }

        bool isDepthPositive() {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            return (v1->estimate().map(Xw))(2)>0.0;
        }


        virtual void linearizeOplus();

        Vector3d cam_project(const Vector3d & trans_xyz) const;

        Vector3d Xw;
        double fx, fy, cx, cy, bf;
    };



} // end namespace

#endif //ORB_SLAM2_KINETIC_TYPES_SIX_DOF_EXPMAP_H
