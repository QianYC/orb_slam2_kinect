//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_LINEAR_SOLVE_DENSE_H
#define ORB_SLAM2_KINETIC_LINEAR_SOLVE_DENSE_H

#include "../core/linear_solver.h"
#include "../core/batch_stats.h"

#include <vector>
#include <utility>
#include<Eigen/Core>
#include<Eigen/Cholesky>


namespace g2o {

    /**
     * \brief linear solver using dense cholesky decomposition
     */
    template <typename MatrixType>
    class LinearSolverDense : public LinearSolver<MatrixType>
    {
    public:
        LinearSolverDense() :
                LinearSolver<MatrixType>(),
                _reset(true)
        {
        }

        virtual ~LinearSolverDense()
        {
        }

        virtual bool init()
        {
            _reset = true;
            return true;
        }

        bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b)
        {
            int n = A.cols();
            int m = A.cols();

            Eigen::MatrixXd& H = _H;
            if (H.cols() != n) {
                H.resize(n, m);
                _reset = true;
            }
            if (_reset) {
                _reset = false;
                H.setZero();
            }

            // copy the sparse block matrix into a dense matrix
            int c_idx = 0;
            for (size_t i = 0; i < A.blockCols().size(); ++i) {
                int c_size = A.colsOfBlock(i);
                assert(c_idx == A.colBaseOfBlock(i) && "mismatch in block indices");

                const typename SparseBlockMatrix<MatrixType>::IntBlockMap& col = A.blockCols()[i];
                if (col.size() > 0) {
                    typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it;
                    for (it = col.begin(); it != col.end(); ++it) {
                        int r_idx = A.rowBaseOfBlock(it->first);
                        // only the upper triangular block is processed
                        if (it->first <= (int)i) {
                            int r_size = A.rowsOfBlock(it->first);
                            H.block(r_idx, c_idx, r_size, c_size) = *(it->second);
                            if (r_idx != c_idx) // write the lower triangular block
                                H.block(c_idx, r_idx, c_size, r_size) = it->second->transpose();
                        }
                    }
                }

                c_idx += c_size;
            }

            // solving via Cholesky decomposition
            Eigen::VectorXd::MapType xvec(x, m);
            Eigen::VectorXd::ConstMapType bvec(b, n);
            _cholesky.compute(H);
            if (_cholesky.isPositive()) {
                xvec = _cholesky.solve(bvec);
                return true;
            }
            return false;
        }

    protected:
        bool _reset;
        Eigen::MatrixXd _H;
        Eigen::LDLT<Eigen::MatrixXd> _cholesky;

    };


}// end namespace


#endif //ORB_SLAM2_KINETIC_LINEAR_SOLVE_DENSE_H
