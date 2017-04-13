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

#include "Frame.h"
#include <thread>

namespace sample_ORB
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor

Frame::Frame(const Frame &frame)
    :mpORBextractor(frame.mpORBextractor),
     mTimeStamp(frame.mTimeStamp),
     mK(frame.mK.clone()), 
     mDistCoef(frame.mDistCoef.clone()),
     N(frame.N),
     mvKeys(frame.mvKeys), 
     mvKeysUn(frame.mvKeysUn),
     mDescriptors(frame.mDescriptors.clone()),
     mvpMapPoints(frame.mvpMapPoints), 
     mvbOutlier(frame.mvbOutlier), 
     mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), 
     mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), 
     mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), 
     mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), 
     mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, cv::Mat &K, cv::Mat &distCoef)
    :mpORBextractor(extractor),
     mTimeStamp(timeStamp), 
     mK(K.clone()),
     mDistCoef(distCoef.clone())
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractor->GetLevels();
    mfScaleFactor = mpORBextractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractor->GetScaleFactors();
    mvInvScaleFactors = mpORBextractor->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractor->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();


    mvpMapPoints = std::vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = std::vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    AssignFeaturesToGrid();
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mtwc = -mRcw.t()*mtcw;
}

void Frame::ExtractORB(const cv::Mat &im)
{
    (*mpORBextractor)(im,cv::Mat(),mvKeys,mDescriptors);
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imGray)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0;         mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imGray.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0;         mat.at<float>(2,1)=imGray.rows;
        mat.at<float>(3,0)=imGray.cols; mat.at<float>(3,1)=imGray.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = std::min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = std::max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = std::min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = std::max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imGray.cols;
        mnMinY = 0.0f;
        mnMaxY = imGray.rows;
    }
}


void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


} //namespace ORB_SLAM
