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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<string>
#include "Drawer.h"
#include "LocalMapping.h"
#include "Frame.h"
#include "ORBextractor.h"
#include <tf/transform_broadcaster.h>

#include <mutex>

namespace sample_ORB
{

class Drawer;
class Map;
class LocalMapping;

class Tracking
{  

public:
    Tracking(Drawer* pDrawer,
              const std::string &strSettingPath);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImage(const cv::Mat &im, tf::Transform odometryTransform ,const double &timestamp, const double &dt);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    std::list<KeyFrame*> GetLocalKeyFrames();

    std::vector<MapPoint*> GetLocalMapPoints();



    

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        TRY_INITIALIZE=2,
        OK=3,
        LOST=4
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    void Initialization();
    void CreateInitialMap();

    bool TrackReferenceKeyFrame();
    //void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool NeedNewKeyFrame();

    void CreateNewKeyFrame();

    void CreateNewMapPoints();

    bool TrackLocalMap();

    void SearchLocalPoints();

    void UpdateLocalPoints();

    void PoseUpdateByOdom(cv::Mat &Tcw, const cv::Vec3f odom);

/*
    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();




*/
    //Other Thread Pointers
    LocalMapping* mpLocalMapper;

    //ORB
    ORBextractor* mpORBextractor;
    ORBextractor* mpIniORBextractor;


    //Local Map
    KeyFrame* mpReferenceKF;
    std::list<KeyFrame*> mlpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    //Drawers
    Drawer* mpDrawer;


    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    std::mutex mMutexLocalKeyFrames;

    cv::Vec3f mDiffPose;

    float mDt;

    cv::Mat mTcwOdom;
    cv::Mat mTcwInit;

    cv::Mat mCampose;

    bool mbTryInit;

    tf::Transform mCurrOdometryTransform;
    tf::Transform mLastOdometryTransform;

};

} //namespace ORB_SLAM

#endif // TRACKING_H
