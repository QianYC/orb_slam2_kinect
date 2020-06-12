//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_MATRIX_STRUCTURE_H
#define ORB_SLAM2_KINETIC_MATRIX_STRUCTURE_H

namespace g2o {

/**
 * \brief representing the structure of a matrix in column compressed structure (only the upper triangular part of the matrix)
 */
    class MatrixStructure
    {
    public:
        MatrixStructure();
        ~MatrixStructure();
        /**
         * allocate space for the Matrix Structure. You may call this on an already allocated struct, it will
         * then reallocate the memory + additional space (double the required space).
         */
        void alloc(int n_, int nz);

        void free();

        /**
         * Write the matrix pattern to a file. File is also loadable by octave, e.g., then use spy(matrix)
         */
        bool write(const char* filename) const;

        int n;    ///< A is m-by-n.  n must be >= 0.
        int m;    ///< A is m-by-n.  m must be >= 0.
        int* Ap;  ///< column pointers for A, of size n+1
        int* Aii; ///< row indices of A, of size nz = Ap [n]

        //! max number of non-zeros blocks
        int nzMax() const { return maxNz;}

    protected:
        int maxN;     ///< size of the allocated memory
        int maxNz;    ///< size of the allocated memory
    };

} // end namespace

#endif //ORB_SLAM2_KINETIC_MATRIX_STRUCTURE_H
