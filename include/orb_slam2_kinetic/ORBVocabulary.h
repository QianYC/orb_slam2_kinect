//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_ORBVOCABULARY_H
#define ORB_SLAM2_KINETIC_ORBVOCABULARY_H

#include"third_party/DBoW2/DBoW2/FORB.h"
#include"third_party/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2
{

    typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
            ORBVocabulary;

} //namespace ORB_SLAM

#endif //ORB_SLAM2_KINETIC_ORBVOCABULARY_H
