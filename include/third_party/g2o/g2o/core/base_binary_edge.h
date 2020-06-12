//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_BASE_BINARY_EDGE_H
#define ORB_SLAM2_KINETIC_BASE_BINARY_EDGE_H

#include <iostream>
#include <limits>

#include "base_edge.h"
#include "robust_kernel.h"
#include "../../config.h"

namespace g2o {

    using namespace Eigen;

    template <int D, typename E, typename VertexXi, typename VertexXj>
    class BaseBinaryEdge : public BaseEdge<D, E>
    {
    public:

        typedef VertexXi VertexXiType;
        typedef VertexXj VertexXjType;

        static const int Di = VertexXiType::Dimension;
        static const int Dj = VertexXjType::Dimension;

        static const int Dimension = BaseEdge<D, E>::Dimension;
        typedef typename BaseEdge<D,E>::Measurement Measurement;
        typedef typename Matrix<double, D, Di>::AlignedMapType JacobianXiOplusType;
        typedef typename Matrix<double, D, Dj>::AlignedMapType JacobianXjOplusType;
        typedef typename BaseEdge<D,E>::ErrorVector ErrorVector;
        typedef typename BaseEdge<D,E>::InformationType InformationType;

        typedef Eigen::Map<Matrix<double, Di, Dj>, Matrix<double, Di, Dj>::Flags & 0x80 ? Aligned : Unaligned > HessianBlockType;
        typedef Eigen::Map<Matrix<double, Dj, Di>, Matrix<double, Dj, Di>::Flags & 0x80 ? Aligned : Unaligned > HessianBlockTransposedType;

        BaseBinaryEdge() : BaseEdge<D,E>(),
                           _hessianRowMajor(false),
                           _hessian(0, VertexXiType::Dimension, VertexXjType::Dimension), // HACK we map to the null pointer for initializing the Maps
                           _hessianTransposed(0, VertexXjType::Dimension, VertexXiType::Dimension),
                           _jacobianOplusXi(0, D, Di), _jacobianOplusXj(0, D, Dj)
        {
            _vertices.resize(2);
        }

        virtual OptimizableGraph::Vertex* createFrom();
        virtual OptimizableGraph::Vertex* createTo();

        virtual void resize(size_t size);

        virtual bool allVerticesFixed() const;

        virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace);

        /**
         * Linearizes the oplus operator in the vertex, and stores
         * the result in temporary variables _jacobianOplusXi and _jacobianOplusXj
         */
        virtual void linearizeOplus();

        //! returns the result of the linearization in the manifold space for the node xi
        const JacobianXiOplusType& jacobianOplusXi() const { return _jacobianOplusXi;}
        //! returns the result of the linearization in the manifold space for the node xj
        const JacobianXjOplusType& jacobianOplusXj() const { return _jacobianOplusXj;}

        virtual void constructQuadraticForm() ;

        virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor);

        using BaseEdge<D,E>::resize;
        using BaseEdge<D,E>::computeError;

    protected:
        using BaseEdge<D,E>::_measurement;
        using BaseEdge<D,E>::_information;
        using BaseEdge<D,E>::_error;
        using BaseEdge<D,E>::_vertices;
        using BaseEdge<D,E>::_dimension;

        bool _hessianRowMajor;
        HessianBlockType _hessian;
        HessianBlockTransposedType _hessianTransposed;
        JacobianXiOplusType _jacobianOplusXi;
        JacobianXjOplusType _jacobianOplusXj;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

#include "base_binary_edge.hpp"

} // end namespace g2o

#endif //ORB_SLAM2_KINETIC_BASE_BINARY_EDGE_H
