//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_MATRIX_OPERATIONS_H
#define ORB_SLAM2_KINETIC_MATRIX_OPERATIONS_H

#include <Eigen/Core>

namespace g2o {
    namespace internal {

        template<typename MatrixType>
        inline void axpy(const MatrixType& A, const Eigen::Map<const Eigen::VectorXd>& x, int xoff, Eigen::Map<Eigen::VectorXd>& y, int yoff)
        {
            y.segment<MatrixType::RowsAtCompileTime>(yoff) += A * x.segment<MatrixType::ColsAtCompileTime>(xoff);
        }

        template<int t>
        inline void axpy(const Eigen::Matrix<double, Eigen::Dynamic, t>& A, const Eigen::Map<const Eigen::VectorXd>& x, int xoff, Eigen::Map<Eigen::VectorXd>& y, int yoff)
        {
            y.segment(yoff, A.rows()) += A * x.segment<Eigen::Matrix<double, Eigen::Dynamic, t>::ColsAtCompileTime>(xoff);
        }

        template<>
        inline void axpy(const Eigen::MatrixXd& A, const Eigen::Map<const Eigen::VectorXd>& x, int xoff, Eigen::Map<Eigen::VectorXd>& y, int yoff)
        {
            y.segment(yoff, A.rows()) += A * x.segment(xoff, A.cols());
        }

        template<typename MatrixType>
        inline void atxpy(const MatrixType& A, const Eigen::Map<const Eigen::VectorXd>& x, int xoff, Eigen::Map<Eigen::VectorXd>& y, int yoff)
        {
            y.segment<MatrixType::ColsAtCompileTime>(yoff) += A.transpose() * x.segment<MatrixType::RowsAtCompileTime>(xoff);
        }

        template<int t>
        inline void atxpy(const Eigen::Matrix<double, Eigen::Dynamic, t>& A, const Eigen::Map<const Eigen::VectorXd>& x, int xoff, Eigen::Map<Eigen::VectorXd>& y, int yoff)
        {
            y.segment<Eigen::Matrix<double, Eigen::Dynamic, t>::ColsAtCompileTime>(yoff) += A.transpose() * x.segment(xoff, A.rows());
        }

        template<>
        inline void atxpy(const Eigen::MatrixXd& A, const Eigen::Map<const Eigen::VectorXd>& x, int xoff, Eigen::Map<Eigen::VectorXd>& y, int yoff)
        {
            y.segment(yoff, A.cols()) += A.transpose() * x.segment(xoff, A.rows());
        }

    } // end namespace internal
} // end namespace g2o

#endif //ORB_SLAM2_KINETIC_MATRIX_OPERATIONS_H
