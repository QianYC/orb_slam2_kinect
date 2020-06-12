//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_KEYFRAMEDATABASE_H
#define ORB_SLAM2_KINETIC_KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>


namespace ORB_SLAM2
{

    class KeyFrame;
    class Frame;


    class KeyFrameDatabase
    {
    public:

        KeyFrameDatabase(const ORBVocabulary &voc);

        void add(KeyFrame* pKF);

        void erase(KeyFrame* pKF);

        void clear();

        // Loop Detection
        std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

        // Relocalization
        std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

    protected:

        // Associated vocabulary
        const ORBVocabulary* mpVoc;

        // Inverted file
        std::vector<list<KeyFrame*> > mvInvertedFile;

        // Mutex
        std::mutex mMutex;
    };

} //namespace ORB_SLAM

#endif //ORB_SLAM2_KINETIC_KEYFRAMEDATABASE_H
