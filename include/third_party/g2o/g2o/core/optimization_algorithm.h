//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_OPTIMIZATION_ALGORITHM_H
#define ORB_SLAM2_KINETIC_OPTIMIZATION_ALGORITHM_H

#include <vector>
#include <utility>
#include <iosfwd>

#include "../stuff/property.h"

#include "hyper_graph.h"
#include "sparse_block_matrix.h"

namespace g2o {

    class SparseOptimizer;

    /**
     * \brief Generic interface for a non-linear solver operating on a graph
     */
    class  OptimizationAlgorithm
    {
    public:
        enum  SolverResult {Terminate=2, OK=1, Fail=-1};
        OptimizationAlgorithm();
        virtual ~OptimizationAlgorithm();

        /**
         * initialize the solver, called once before the first call to solve()
         */
        virtual bool init(bool online = false) = 0;

        /**
         * Solve one iteration. The SparseOptimizer running on-top will call this
         * for the given number of iterations.
         * @param iteration indicates the current iteration
         */
        virtual SolverResult solve(int iteration, bool online = false) = 0;

        /**
         * computes the block diagonal elements of the pattern specified in the input
         * and stores them in given SparseBlockMatrix.
         * If your solver does not support computing the marginals, return false.
         */
        virtual bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices) = 0;

        /**
         * update the structures for online processing
         */
        virtual bool updateStructure(const std::vector<HyperGraph::Vertex*>& vset, const HyperGraph::EdgeSet& edges) = 0;

        /**
         * called by the optimizer if verbose. re-implement, if you want to print something
         */
        virtual void printVerbose(std::ostream& os) const {(void) os;};

    public:
        //! return the optimizer operating on
        const SparseOptimizer* optimizer() const { return _optimizer;}
        SparseOptimizer* optimizer() { return _optimizer;}

        /**
         * specify on which optimizer the solver should work on
         */
        void setOptimizer(SparseOptimizer* optimizer);

        //! return the properties of the solver
        const PropertyMap& properties() const { return _properties;}

        /**
         * update the properties from a string, see PropertyMap::updateMapFromString()
         */
        bool updatePropertiesFromString(const std::string& propString);

        /**
         * print the properties to a stream in a human readable fashion
         */
        void printProperties(std::ostream& os) const;

    protected:
        SparseOptimizer* _optimizer;   ///< the optimizer the solver is working on
        PropertyMap _properties;       ///< the properties of your solver, use this to store the parameters of your solver

    private:
        // Disable the copy constructor and assignment operator
        OptimizationAlgorithm(const OptimizationAlgorithm&) { }
        OptimizationAlgorithm& operator= (const OptimizationAlgorithm&) { return *this; }
    };

} // end namespace

#endif //ORB_SLAM2_KINETIC_OPTIMIZATION_ALGORITHM_H
