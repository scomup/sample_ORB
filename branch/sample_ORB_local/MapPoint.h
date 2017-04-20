/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"

#include <vector>
#include <list>
#include <set>
#include <map>

#include<opencv2/core/core.hpp>
#include<mutex>

namespace sample_ORB
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF);
    MapPoint(const cv::Mat &Pos, Frame* pFrame, const int &idxF);
    void AddObservation(KeyFrame* pKF,size_t idx);
    void ComputeDistinctiveDescriptors();
    void UpdateNormalAndDepth();
    std::map<KeyFrame*,size_t> GetObservations();
    cv::Mat GetWorldPos();
    cv::Mat GetDescriptor();
    bool IsInKeyFrame(KeyFrame* pKF);
    cv::Mat GetNormal();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int GetIndexInKeyFrame(KeyFrame* pKF);
    void SetWorldPos(const cv::Mat &Pos);
    int PredictScale(const float &currentDist, Frame* pF);
    int PredictScale(const float &currentDist, KeyFrame* pKF);
    void EraseObservation(KeyFrame* pKF);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnLastFrameSeen;
    long unsigned int mnTrackReferenceForFrame;
    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    
    static std::mutex mGlobalMutex;

protected:    

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
