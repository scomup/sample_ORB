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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Tracking.h"

#include <mutex>


namespace sample_ORB
{
    using namespace std;

class Tracking;
class Map;

class LocalMapping
{
public:
    LocalMapping();

    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    void SetTracker(Tracking *pTracker);

    void SetFinish();

    bool isFinished();

    

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();



    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    std::list<KeyFrame*> mlpLocalKeyFrames;

    std::vector<MapPoint*> mvpLocalMapPoints;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::mutex mMutexNewKFs;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    bool mbStopped;
    std::mutex mMutexStop;

    Tracking* mpTracker;

    bool mbFinished;
    std::mutex mMutexFinish;



};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
