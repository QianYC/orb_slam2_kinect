//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_MAP_H
#define ORB_SLAM2_KINETIC_MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>



namespace ORB_SLAM2
{

    class MapPoint;
    class KeyFrame;

    class Map
    {
    public:
        Map();

        void AddKeyFrame(KeyFrame* pKF);
        void AddMapPoint(MapPoint* pMP);
        void EraseMapPoint(MapPoint* pMP);
        void EraseKeyFrame(KeyFrame* pKF);
        void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
        void InformNewBigChange();
        int GetLastBigChangeIdx();

        std::vector<KeyFrame*> GetAllKeyFrames();
        std::vector<MapPoint*> GetAllMapPoints();
        std::vector<MapPoint*> GetReferenceMapPoints();

        long unsigned int MapPointsInMap();
        long unsigned  KeyFramesInMap();

        long unsigned int GetMaxKFid();

        void clear();

        vector<KeyFrame*> mvpKeyFrameOrigins;

        std::mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;

    protected:
        std::set<MapPoint*> mspMapPoints;
        std::set<KeyFrame*> mspKeyFrames;

        std::vector<MapPoint*> mvpReferenceMapPoints;

        long unsigned int mnMaxKFid;

        // Index related to a big change in the map (loop closure, global BA)
        int mnBigChangeIdx;

        std::mutex mMutexMap;
    };

} //namespace ORB_SLAM

#endif //ORB_SLAM2_KINETIC_MAP_H
