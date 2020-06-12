//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_SOLVER_H
#define ORB_SLAM2_KINETIC_SOLVER_H

#include "hyper_graph.h"
#include "batch_stats.h"
#include "sparse_block_matrix.h"
#include <cstddef>

namespace g2o {


    class SparseOptimizer;

    /**
     * \brief Generic interface for a sparse solver operating on a graph which solves one iteration of the linearized objective function
     */
    class  Solver
    {
    public:
        Solver();
        virtual ~Solver();

    public:
        /**
         * initialize the solver, called once before the first iteration
         */
        virtual bool init(SparseOptimizer* optimizer, bool online = false) = 0;

        /**
         * build the structure of the system
         */
        virtual bool buildStructure(bool zeroBlocks = false) = 0;
        /**
         * update the structures for online processing
         */
        virtual bool updateStructure(const std::vector<HyperGraph::Vertex*>& vset, const HyperGraph::EdgeSet& edges) = 0;
        /**
         * build the current system
         */
        virtual bool buildSystem() = 0;

        /**
         * solve Ax = b
         */
        virtual bool solve() = 0;

        /**
         * computes the block diagonal elements of the pattern specified in the input
         * and stores them in given SparseBlockMatrix
         */
        virtual bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices) = 0;

        /**
         * update the system while performing Levenberg, i.e., modifying the diagonal
         * components of A by doing += lambda along the main diagonal of the Matrix.
         * Note that this function may be called with a positive and a negative lambda.
         * The latter is used to undo a former modification.
         * If backup is true, then the solver should store a backup of the diagonal, which
         * can be restored by restoreDiagonal()
         */
        virtual bool setLambda(double lambda, bool backup = false) = 0;

        /**
         * restore a previosly made backup of the diagonal
         */
        virtual void restoreDiagonal() = 0;

        //! return x, the solution vector
        double* x() { return _x;}
        const double* x() const { return _x;}
        //! return b, the right hand side of the system
        double* b() { return _b;}
        const double* b() const { return _b;}

        //! return the size of the solution vector (x) and b
        size_t vectorSize() const { return _xSize;}

        //! the optimizer (graph) on which the solver works
        SparseOptimizer* optimizer() const { return _optimizer;}
        void setOptimizer(SparseOptimizer* optimizer);

        //! the system is Levenberg-Marquardt
        bool levenberg() const { return _isLevenberg;}
        void setLevenberg(bool levenberg);

        /**
         * does this solver support the Schur complement for solving a system consisting of poses and
         * landmarks. Re-implemement in a derived solver, if your solver supports it.
         */
        virtual bool supportsSchur() {return false;}

        //! should the solver perform the schur complement or not
        virtual bool schur()=0;
        virtual void setSchur(bool s)=0;

        size_t additionalVectorSpace() const { return _additionalVectorSpace;}
        void setAdditionalVectorSpace(size_t s);

        /**
         * write debug output of the Hessian if system is not positive definite
         */
        virtual void setWriteDebug(bool) = 0;
        virtual bool writeDebug() const = 0;

        //! write the hessian to disk using the specified file name
        virtual bool saveHessian(const std::string& /*fileName*/) const = 0;

    protected:
        SparseOptimizer* _optimizer;
        double* _x;
        double* _b;
        size_t _xSize, _maxXSize;
        bool _isLevenberg; ///< the system we gonna solve is a Levenberg-Marquardt system
        size_t _additionalVectorSpace;

        void resizeVector(size_t sx);

    private:
        // Disable the copy constructor and assignment operator
        Solver(const Solver&) { }
        Solver& operator= (const Solver&) { return *this; }
    };
} // end namespace

#endif //ORB_SLAM2_KINETIC_SOLVER_H
