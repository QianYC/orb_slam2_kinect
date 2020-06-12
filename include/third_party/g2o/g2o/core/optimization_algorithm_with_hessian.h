//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_OPTIMIZATION_ALGORITHM_WITH_HESSIAN_H
#define ORB_SLAM2_KINETIC_OPTIMIZATION_ALGORITHM_WITH_HESSIAN_H

#include "optimization_algorithm.h"

namespace g2o {

    class Solver;

    /**
     * \brief Base for solvers operating on the approximated Hessian, e.g., Gauss-Newton, Levenberg
     */
    class  OptimizationAlgorithmWithHessian : public OptimizationAlgorithm
    {
    public:
        explicit OptimizationAlgorithmWithHessian(Solver* solver);
        virtual ~OptimizationAlgorithmWithHessian();

        virtual bool init(bool online = false);

        virtual bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices);

        virtual bool buildLinearStructure();

        virtual void updateLinearSystem();

        virtual bool updateStructure(const std::vector<HyperGraph::Vertex*>& vset, const HyperGraph::EdgeSet& edges);

        //! return the underlying solver used to solve the linear system
        Solver* solver() { return _solver;}

        /**
         * write debug output of the Hessian if system is not positive definite
         */
        virtual void setWriteDebug(bool writeDebug);
        virtual bool writeDebug() const { return _writeDebug->value();}

    protected:
        Solver* _solver;
        Property<bool>* _writeDebug;

    };

}// end namespace

#endif //ORB_SLAM2_KINETIC_OPTIMIZATION_ALGORITHM_WITH_HESSIAN_H
