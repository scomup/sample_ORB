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

#include "KeyFrame.h"
#include "ORBmatcher.h"
#include<mutex>

namespace sample_ORB
{
    using namespace std;

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F/*, Map *pMap = nullptr*/):
    mnFrameId(F.mnId),  
    mTimeStamp(F.mTimeStamp), 
    mnGridCols(FRAME_GRID_COLS), 
    mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), 
    mfGridElementHeightInv(F.mfGridElementHeightInv),
    //mnTrackReferenceForFrame(0), 
    //mnFuseTargetForKF(0), 
    //mnBALocalForKF(0), 
    //mnBAFixedForKF(0),
    fx(F.fx), 
    fy(F.fy), 
    cx(F.cx), 
    cy(F.cy), 
    invfx(F.invfx), 
    invfy(F.invfy),
    N(F.N), 
    mvKeys(F.mvKeys), 
    mvKeysUn(F.mvKeysUn),
    mDescriptors(F.mDescriptors.clone()),
    mnScaleLevels(F.mnScaleLevels), 
    mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), 
    mvScaleFactors(F.mvScaleFactors), 
    mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), 
    mnMinX(F.mnMinX), 
    mnMinY(F.mnMinY), 
    mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), 
    mK(F.mK), 
    mvpMapPoints(F.mvpMapPoints), 
    mbFirstConnection(true), 
    mpParent(NULL),
    //mbNotErase(false),
    //mbToBeErased(false), 
    mbBad(false) 
    //mpMap(pMap)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}


void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    twc = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    twc.copyTo(Twc.rowRange(0,3).col(3));
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return twc.clone();
}

cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}



} //namespace ORB_SLAM
