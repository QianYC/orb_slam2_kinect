//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_BASE_EDGE_H
#define ORB_SLAM2_KINETIC_BASE_EDGE_H

#include <iostream>
#include <limits>

#include <Eigen/Core>

#include "optimizable_graph.h"

namespace g2o {

    using namespace Eigen;

    template <int D, typename E>
    class BaseEdge : public OptimizableGraph::Edge
    {
    public:

        static const int Dimension = D;
        typedef E Measurement;
        typedef Matrix<double, D, 1> ErrorVector;
        typedef Matrix<double, D, D> InformationType;

        BaseEdge() : OptimizableGraph::Edge()
        {
            _dimension = D;
        }

        virtual ~BaseEdge() {}

        virtual double chi2() const
        {
            return _error.dot(information()*_error);
        }

        virtual const double* errorData() const { return _error.data();}
        virtual double* errorData() { return _error.data();}
        const ErrorVector& error() const { return _error;}
        ErrorVector& error() { return _error;}

        //! information matrix of the constraint
        const InformationType& information() const { return _information;}
        InformationType& information() { return _information;}
        void setInformation(const InformationType& information) { _information = information;}

        virtual const double* informationData() const { return _information.data();}
        virtual double* informationData() { return _information.data();}

        //! accessor functions for the measurement represented by the edge
        const Measurement& measurement() const { return _measurement;}
        virtual void setMeasurement(const Measurement& m) { _measurement = m;}

        virtual int rank() const {return _dimension;}

        virtual void initialEstimate(const OptimizableGraph::VertexSet&, OptimizableGraph::Vertex*)
        {
            std::cerr << "inititialEstimate() is not implemented, please give implementation in your derived class" << std::endl;
        }

    protected:

        Measurement _measurement;
        InformationType _information;
        ErrorVector _error;

        /**
         * calculate the robust information matrix by updating the information matrix of the error
         */
        InformationType robustInformation(const Eigen::Vector3d& rho)
        {
            InformationType result = rho[1] * _information;
            //ErrorVector weightedErrror = _information * _error;
            //result.noalias() += 2 * rho[2] * (weightedErrror * weightedErrror.transpose());
            return result;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // end namespace g2o

#endif //ORB_SLAM2_KINETIC_BASE_EDGE_H
