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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"Drawer.h"
#include"Optimizer.h"
#include "Converter.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace sample_ORB
{

Tracking::Tracking(Drawer *pDrawer,  const string &strSettingPath):
    mState(NO_IMAGES_YET),
    mpDrawer(pDrawer)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);


    mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

}


void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}



cv::Mat Tracking::GrabImage(const cv::Mat &im, cv::Vec3f odom, const double &timestamp)
{
    mImGray = im;

    if (!mLastFrame.mTcw.empty())
    {
        cv::Mat Rcw = mLastFrame.mTcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat v = (cv::Mat_<float>(3, 1) << odom[0], odom[1], 0);
        v = Rcw * v;
        printf("odom:dx:%f dy:%f dyaw%f\n", v.at<float>(0), v.at<float>(1), odom[2]);
    }
    //float pitch, roll, yaw;
    //Converter::computeAnglesFromMatrix(Rcw, pitch, roll, yaw);
    //cv::Mat Ric = Converter::computeMatrixFromAngles(0, 0, mDiffPose[2]);


    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,odom,timestamp,mpIniORBextractor,mK,mDistCoef);
    else
        mCurrentFrame = Frame(mImGray,odom,timestamp,mpORBextractor,mK,mDistCoef);

    Track();
    cv::Mat sim = im.clone();
    for (size_t i = 0; i < mCurrentFrame.mvpMapPoints.size(); i++)
    {
        if(mCurrentFrame.mvpMapPoints[i] != NULL){
            cv::KeyPoint kp = mCurrentFrame.mvKeys[i];
            cv::circle(sim, kp.pt, 2,cv::Scalar(0, 255, 0));
        }
    }
    cv::imshow("FRAME", sim);
    cv::waitKey(1);

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;


    if(mState==NOT_INITIALIZED)
    {
        
        Initialization();

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        if (mState == OK)
        {
            if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
            {
                bOK = TrackReferenceKeyFrame();
            }
            else
            {
                //bOK = TrackReferenceKeyFrame();
                bOK = TrackWithMotionModel();
                if (!bOK)
                {
                    printf("TrackReferenceKeyFrame!\n");
                    bOK = TrackReferenceKeyFrame();
                }
            }
        }
        
        else
        {
            printf("TODO: pose error!\n");
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        if(bOK)
            bOK = TrackLocalMap();
        else
            printf("TODO: Cannot tracking!\n");


        if(bOK)
            mState = OK;
        else
        {
            mState = LOST;
            printf("TrackLocalMap Error!\n");
        }

        if(bOK)
        {
            // Check if we need to insert a new keyframe

            if(NeedNewKeyFrame())
            {
                CreateNewKeyFrame();
            }
            else
            {
                /*
                map<KeyFrame *, int> KFcounter;
                for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
                {
                    MapPoint *pMP = *vit;

                    if (!pMP)
                        continue;

                    map<KeyFrame *, size_t> observations = pMP->GetObservations();

                    for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
                    {
                        KFcounter[mit->first]++;
                    }
                }
                KeyFrame *currentBestKF = 0;
                int currentMax = 0;
                for (auto it = KFcounter.begin(); it != KFcounter.end(); ++it)
                {

                    if (it->second > currentMax)
                    {
                        currentBestKF = it->first;
                        currentMax = it->second;
                    }
                }
                mpReferenceKF = currentBestKF;
                */
                
        }

                // If tracking were good, check if we insert a keyframe
        if (bOK)
        {
            // Update motion model
            if (!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                mVelocity = mCurrentFrame.mTcw * LastTwc;

            }
            else
                mVelocity = cv::Mat();
        }

        // We allow points with high innovation (considererd outliers by the Huber Function)
        // pass to the new keyframe, so that bundle adjustment will finally decide
        // if they are outliers or not. We don't want next frame to estimate its position
        // with those points so we discard them in the frame.
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            cout << "TODO: initialization reseting..." << endl;
            exit(0);
            return;
            
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }
    mpDrawer->SetDrawer(mCurrentFrame.mTcw, mlpLocalKeyFrames, mvpLocalMapPoints);

}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    //cout<<mVelocity<<endl;
    //cout<<mLastFrame.mTcw<<endl;
    //cout<<mVelocity*mLastFrame.mTcw<<endl;
    //cv::Mat V = mVelocity.clone();
    //V= V*mLastFrame.mTcw;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th = 7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th);

    // If few matches, uses a wider window search
    if(nmatches<100)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th);
    }

    if(nmatches<50)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
                nmatchesMap++;
        }
    }    

    return nmatchesMap>=30;
}

void Tracking::Initialization()
{
    static bool flag = false;

    if (!flag)
    {
        // Set Reference Frame
        mDiffPose[0] = 0;
        mDiffPose[1] = 0;
        mDiffPose[2] = 0;
        if (mCurrentFrame.mvKeys.size() > 100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;


            flag = true;

            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if ((int)mCurrentFrame.mvKeys.size() <= 50)
        {
            flag = false;
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.7, true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

        // Check if there are enough correspondences
        if (nmatches < 100)
        {
            mState = NOT_INITIALIZED;
            return;
        }
        mDiffPose[2] += mCurrentFrame.mOdom[2];

        cv::Mat Ric = Converter::computeMatrixFromAngles(0, 0, mDiffPose[2]);
        cv::Mat dtic = Ric*(cv::Mat_<float>(3,1) << mCurrentFrame.mOdom[0], mCurrentFrame.mOdom[1], 0);
        mDiffPose[0] +=dtic.at<float>(0);
        mDiffPose[1] +=dtic.at<float>(1);
        cout<<mDiffPose<<endl;

        cv::Mat Rcw2 = Ric.t();
        cv::Mat Rwc2 = Ric;
        cv::Mat tcw2 = -Rcw2*(cv::Mat_<float>(3,1) << mDiffPose[0], mDiffPose[1], 0);

        cv::Mat Tcw2 = cv::Mat::eye(4,4,CV_32F);
        Rcw2.copyTo(Tcw2.rowRange(0,3).colRange(0, 3));
        tcw2.copyTo(Tcw2.rowRange(0, 3).col(3));

        cv::Mat Rcw1 = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat Rwc1 = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat tcw1 = cv::Mat::zeros(3, 1, CV_32F);
        cv::Mat Tcw1 = cv::Mat::eye(4, 4, CV_32F);

        const float fx2 = mCurrentFrame.fx;
        const float fy2 = mCurrentFrame.fy;
        const float cx2 = mCurrentFrame.cx;
        const float cy2 = mCurrentFrame.cy;
        const float invfx2 = 1.0f / fx2;
        const float invfy2 = 1.0f / fy2;

        const float fx1 = mInitialFrame.fx;
        const float fy1 = mInitialFrame.fy;
        const float cx1 = mInitialFrame.cx;
        const float cy1 = mInitialFrame.cy;
        const float invfx1 = 1.0f / fx1;
        const float invfy1 = 1.0f / fy1;

        // Triangulate each match
        //mInitialFrame,mCurrentFrame
        //   SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
        double base_line = cv::norm(tcw2);
        //printf("%f\n",base_line);
        //printf("tfi->x:%5.3f y:%5.3f z:%5.3f yaw:%5.3f\n",-transpose.y(), transpose.x(), transpose.z(), mCurrentFrame.mYaw);
        if (base_line > 0.2)
        {
            mvIniP3D.resize(mvIniMatches.size());
            for (size_t ikp = 0, iendkp = mvIniMatches.size(); ikp < iendkp; ikp++)
            {
                if (mvIniMatches[ikp] == -1)
                    continue;

                const cv::KeyPoint &kp1 = mInitialFrame.mvKeysUn[ikp];
                const cv::KeyPoint &kp2 = mCurrentFrame.mvKeysUn[mvIniMatches[ikp]];
                float diff = (kp1.pt.x - kp2.pt.x)*(kp1.pt.x - kp2.pt.x) + (kp1.pt.y - kp2.pt.y)*(kp1.pt.y - kp2.pt.y);
                if(diff < 16)
                {
                    mvIniMatches[ikp]=-1;
                    continue;
                }

                //cv::line(mInitialFrame.im, kp1.pt, kp2.pt, cv::Scalar(0, 0, 255));
                //cv::line(mCurrentFrame.im, kp1.pt, kp2.pt, cv::Scalar(0, 0, 255));
                //cv::circle(mInitialFrame.im, kp1.pt, 2, cv::Scalar(0, 255, 0));
                //cv::circle(mCurrentFrame.im, kp2.pt, 2, cv::Scalar(0, 255, 0));

                // Check parallax between rays
                cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
                cv::Mat ray1 = Rwc1 * xn1;
                cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2) * invfx2, (kp2.pt.y - cy2) * invfy2, 1.0);
                cv::Mat ray2 = Rwc2 * xn2;
                const float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

                if (cosParallaxRays < 0 || cosParallaxRays > 0.9998)
                {
                    mvIniMatches[ikp]=-1;
                    continue;
                }

                // Linear Triangulation Method
                cv::Mat A(4, 4, CV_32F);
                A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
                A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
                A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
                A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

                cv::Mat w, u, vt;
                cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

                cv::Mat x3D = vt.row(3).t();

                if (x3D.at<float>(3) == 0)
                {
                    mvIniMatches[ikp]=-1;
                    continue;
                }

                // Euclidean coordinates
                x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
                cv::Mat x3Dt = x3D.t();

                //Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
                if (z1 <= 0)
                {
                    mvIniMatches[ikp]=-1;
                    continue;
                }

                float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
                if (z2 <= 0)
                {
                    mvIniMatches[ikp]=-1;
                    continue;
                }


                if (x3Dt.at<float>(2) < 1 && x3Dt.at<float>(2) > 20)
                {
                    mvIniMatches[ikp]=-1;
                    continue;
                }

                //cout<<ikp<<":"<<x3Dt<<endl;
                mvIniP3D[ikp] = cv::Point3f(x3Dt.at<float>(0), x3Dt.at<float>(1), x3Dt.at<float>(2));
                //printf("init point %5.3f %5.3f %5.3f\n",x3Dt.at<float>(0),x3Dt.at<float>(1),x3Dt.at<float>(2));
            }
            //cv::imwrite("left.png",mInitialFrame.im);
            //cv::imwrite("right.png",mCurrentFrame.im);

                        // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            mCurrentFrame.SetPose(Tcw2);

            CreateInitialMap();
            cout << "My Initial OK!" << endl;
        }
    }
}

void Tracking::CreateInitialMap()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);
                
        MapPoint* pMP = new MapPoint(worldPos,pKFcur);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);
        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
    }

    // Update Connections
    pKFcur->ChangeParent(pKFini);

    cout << "New Map created ! " << endl;

    if(pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "TODO: Wrong initialization, reseting..." << endl;
        return;
    }



    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    mlpLocalKeyFrames.push_back(pKFini);
    mlpLocalKeyFrames.push_back(pKFcur);
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;
    mLastFrame = Frame(mCurrentFrame);


    mState=OK;
}

/*
void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}*/


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;
    int nmatches = matcher.SearchNearby(mpReferenceKF, mCurrentFrame);
    //int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
    int nhave = mpReferenceKF->TrackedMapPoints(1);
    printf("have %d points in TrackReferenceKeyFrame\n",nhave);
    printf("nmatches %d in TrackReferenceKeyFrame\n",nmatches);

    if(nmatches<50)
        return false;

    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else
                nmatchesMap++;
        }
    }
  
    return nmatchesMap>=10;
}

bool Tracking::NeedNewKeyFrame()
{

    int nRefMatches = mpReferenceKF->TrackedMapPoints(1);

    const bool c = (mnMatchesInliers<nRefMatches*0.6 );
    if(c && mCurrentFrame.mOdom[2] < 0.001)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Tracking::CreateNewKeyFrame()
{
    //if(!mpLocalMapper->SetNotStop(true))
    //    return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame);
    pKF->ChangeParent(mpReferenceKF);
    mpReferenceKF = pKF;
    //printf("new mpReferenceKF %ld\n",mpReferenceKF->mnId);
    mCurrentFrame.mpReferenceKF = pKF;

    mpLocalMapper->InsertKeyFrame(pKF);

    //mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
    mlpLocalKeyFrames.push_back(pKF);
    if(mlpLocalKeyFrames.size()>10){
        mlpLocalKeyFrames.pop_front();
    }

}

bool Tracking::TrackLocalMap()
{
    //Find all MapPoints in mlpLocalKeyFrames.
    UpdateLocalPoints();
    //Try to match MapPoints to CurrentFrame.
    SearchLocalPoints();

    //Optimize Pose by MapPoints
    Optimizer::PoseOptimization(&mCurrentFrame);

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mnMatchesInliers++;
            }
        }
    }
    
    // Decide if the tracking was succesful
    //printf("mnMatchesInliers:%ld\n",mnMatchesInliers);
    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            pMP->mnLastFrameSeen = mCurrentFrame.mnId;
            pMP->mbTrackInView = false;
            
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}


void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(auto itKF=mlpLocalKeyFrames.begin(), itEndKF=mlpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            mvpLocalMapPoints.push_back(pMP);
            pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            
        }
    }
}


std::list<KeyFrame*> Tracking::GetLocalKeyFrames(){
    //unique_lock<mutex> lock(mMutexNewKFs);
    return mlpLocalKeyFrames;
}

std::vector<MapPoint*> Tracking::GetLocalMapPoints(){
    return mvpLocalMapPoints;
}


} //namespace ORB_SLAM
