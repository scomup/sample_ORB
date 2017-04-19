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

KeyFrame::KeyFrame(Frame &F):
    mnFrameId(F.mnId),  
    mTimeStamp(F.mTimeStamp), 
    mnGridCols(FRAME_GRID_COLS), 
    mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), 
    mfGridElementHeightInv(F.mfGridElementHeightInv),
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
    mpParent(NULL)
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

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
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

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}


KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}


int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            nPoints++;
            
        }
    }

    return nPoints;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}


vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}


} //namespace ORB_SLAM
