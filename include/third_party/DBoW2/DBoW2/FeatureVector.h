//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_FEATUREVECTOR_H
#define ORB_SLAM2_KINETIC_FEATUREVECTOR_H

#include "third_party/DBoW2/DBoW2/BowVector.h"
#include <map>
#include <vector>
#include <iostream>

namespace DBoW2 {

/// Vector of nodes with indexes of local features
    class FeatureVector:
            public std::map<NodeId, std::vector<unsigned int> >
    {
    public:

        /**
         * Constructor
         */
        FeatureVector(void);

        /**
         * Destructor
         */
        ~FeatureVector(void);

        /**
         * Adds a feature to an existing node, or adds a new node with an initial
         * feature
         * @param id node id to add or to modify
         * @param i_feature index of feature to add to the given node
         */
        void addFeature(NodeId id, unsigned int i_feature);

        /**
         * Sends a string versions of the feature vector through the stream
         * @param out stream
         * @param v feature vector
         */
        friend std::ostream& operator<<(std::ostream &out, const FeatureVector &v);

    };

} // namespace DBoW2

#endif //ORB_SLAM2_KINETIC_FEATUREVECTOR_H
