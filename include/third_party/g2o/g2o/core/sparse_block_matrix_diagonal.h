//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_SPARSE_BLOCK_MATRIX_DIAGONAL_H
#define ORB_SLAM2_KINETIC_SPARSE_BLOCK_MATRIX_DIAGONAL_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include "../../config.h"
#include "matrix_operations.h"

namespace g2o {

    /**
     * \brief Sparse matrix which uses blocks on the diagonal
     *
     * This class is used as a const view on a SparseBlockMatrix
     * which allows a faster iteration over the elements of the
     * matrix.
     */
    template <class MatrixType>
    class SparseBlockMatrixDiagonal
    {
    public:
        //! this is the type of the elementary block, it is an Eigen::Matrix.
        typedef MatrixType SparseMatrixBlock;

        //! columns of the matrix
        int cols() const {return _blockIndices.size() ? _blockIndices.back() : 0;}
        //! rows of the matrix
        int rows() const {return _blockIndices.size() ? _blockIndices.back() : 0;}

        typedef std::vector<MatrixType, Eigen::aligned_allocator<MatrixType> >      DiagonalVector;

        SparseBlockMatrixDiagonal(const std::vector<int>& blockIndices) :
                _blockIndices(blockIndices)
        {}

        //! how many rows/cols does the block at block-row / block-column r has?
        inline int dimOfBlock(int r) const { return r ? _blockIndices[r] - _blockIndices[r-1] : _blockIndices[0] ; }

        //! where does the row /col at block-row / block-column r starts?
        inline int baseOfBlock(int r) const { return r ? _blockIndices[r-1] : 0 ; }

        //! the block matrices per block-column
        const DiagonalVector& diagonal() const { return _diagonal;}
        DiagonalVector& diagonal() { return _diagonal;}

        //! indices of the row blocks
        const std::vector<int>& blockIndices() const { return _blockIndices;}

        void multiply(double*& dest, const double* src) const
        {
            int destSize=cols();
            if (! dest) {
                dest=new double[destSize];
                memset(dest,0, destSize*sizeof(double));
            }

            // map the memory by Eigen
            Eigen::Map<Eigen::VectorXd> destVec(dest, destSize);
            Eigen::Map<const Eigen::VectorXd> srcVec(src, rows());

#      ifdef G2O_OPENMP
#      pragma omp parallel for default (shared) schedule(dynamic, 10)
#      endif
            for (int i=0; i < static_cast<int>(_diagonal.size()); ++i){
                int destOffset = baseOfBlock(i);
                int srcOffset = destOffset;
                const SparseMatrixBlock& A = _diagonal[i];
                // destVec += *A.transpose() * srcVec (according to the sub-vector parts)
                internal::axpy(A, srcVec, srcOffset, destVec, destOffset);
            }
        }

    protected:
        const std::vector<int>& _blockIndices; ///< vector of the indices of the blocks along the diagonal
        DiagonalVector _diagonal;
    };

} //end namespace

#endif //ORB_SLAM2_KINETIC_SPARSE_BLOCK_MATRIX_DIAGONAL_H
