//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_JACOBIAN_WORKSPACE_H
#define ORB_SLAM2_KINETIC_JACOBIAN_WORKSPACE_H

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <vector>
#include <cassert>

#include "hyper_graph.h"

namespace g2o {

    struct OptimizableGraph;

    /**
     * \brief provide memory workspace for computing the Jacobians
     *
     * The workspace is used by an OptimizableGraph to provide temporary memory
     * for computing the Jacobian of the error functions.
     * Before calling linearizeOplus on an edge, the workspace needs to be allocated
     * by calling allocate().
     */
    class  JacobianWorkspace
    {
    public:
        typedef std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> >      WorkspaceVector;

    public:
        JacobianWorkspace();
        ~JacobianWorkspace();

        /**
         * allocate the workspace
         */
        bool allocate();

        /**
         * update the maximum required workspace needed by taking into account this edge
         */
        void updateSize(const HyperGraph::Edge* e);

        /**
         * update the required workspace by looking at a full graph
         */
        void updateSize(const OptimizableGraph& graph);

        /**
         * manually update with the given parameters
         */
        void updateSize(int numVertices, int dimension);

        /**
         * return the workspace for a vertex in an edge
         */
        double* workspaceForVertex(int vertexIndex)
        {
            assert(vertexIndex >= 0 && (size_t)vertexIndex < _workspace.size() && "Index out of bounds");
            return _workspace[vertexIndex].data();
        }

    protected:
        WorkspaceVector _workspace;   ///< the memory pre-allocated for computing the Jacobians
        int _maxNumVertices;          ///< the maximum number of vertices connected by a hyper-edge
        int _maxDimension;            ///< the maximum dimension (number of elements) for a Jacobian
    };

} // end namespace


#endif //ORB_SLAM2_KINETIC_JACOBIAN_WORKSPACE_H
