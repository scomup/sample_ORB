#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include "ORBextractor.h"
#include "Frame.h"
#include "ORBmatcher.h"
#include "Initializer.h"
#include "Drawer.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"
#include "CeresPoseSolver.h"
#include "SamplePoseSolver.h"
#include "G2OPoseSolver.h"


#include <iostream>
const char filename_test1[] = "/home/liu/workspace/sample_ORB/data/frame0015.jpg";
const char filename_test2[] = "/home/liu/workspace/sample_ORB/data/frame0018.jpg";
const char filename_test3[] = "/home/liu/workspace/sample_ORB/data/frame0020.jpg";

using namespace cv;
using namespace sample_ORB;
int main()
{
    //==============================================================
    // Input image
    //==============================================================
    Mat input1, input2, input3;
    Mat imGray1, imGray2, imGray3;
    input1 = imread(filename_test1);
    input2 = imread(filename_test2);
    input3 = imread(filename_test3);

    if (input1.empty() || input2.empty() || input3.empty())
    {
        std::cout << "Wrong image path to settings, Please check your yaml file." << std::endl;
        return 1;
    }

    //==============================================================
    // Camera matrix
    //==============================================================
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = 320;
    K.at<float>(1, 1) = 320;
    K.at<float>(0, 2) = 320.5;
    K.at<float>(1, 2) = 240.5;

    //==============================================================
    // Distortion coefficients
    //==============================================================
    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = 0;
    DistCoef.at<float>(1) = 0;
    DistCoef.at<float>(2) = 0;
    DistCoef.at<float>(3) = 0;

    //==============================================================
    // Convert images to Gray
    //==============================================================
    cvtColor(input1, imGray1, CV_RGB2GRAY);
    cvtColor(input2, imGray2, CV_RGB2GRAY);
    cvtColor(input3, imGray3, CV_RGB2GRAY);

    //==============================================================
    // Create Frame
    //==============================================================
    ORBextractor orb = ORBextractor(2000, 1.2, 8, 20, 7);
    Frame frame1 = Frame(imGray1, 0, &orb, K, DistCoef);
    Frame frame2 = Frame(imGray2, 0, &orb, K, DistCoef);
    Frame frame3 = Frame(imGray3, 0, &orb, K, DistCoef);

    //==============================================================
    // Search the matched keyPoints for initialization
    //==============================================================
    ORBmatcher orbMatcher = ORBmatcher(0.7, true);
    std::vector<int> vnMatches12;
    std::vector<cv::Point2f> vbPrevMatched;
    vbPrevMatched.resize(frame1.mvKeysUn.size());
    for (size_t i = 0; i < frame1.mvKeysUn.size(); i++)
        vbPrevMatched[i] = frame1.mvKeysUn[i].pt;
    int n = orbMatcher.SearchForInitialization(frame1, frame2, vbPrevMatched, vnMatches12, 100);
    if ( n < 30)
    {
        printf("Have not enough matched points!(%d)\n", n);
        return 0;
    }

    //==============================================================
    // Try to initialize
    //==============================================================
    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
    std::vector<cv::Point3f> vIniP3D;

    Initializer initializer =  Initializer(frame1, 1.0, 200);
    
    Map map;

    KeyFrame* pKFcurrent;
    
    if(initializer.Initialize(frame2, vnMatches12, Rcw, tcw, vIniP3D, vbTriangulated)){
        printf("Intialized!(%d)\n", n);

    //==============================================================
    // If initialization successed, Try to create map
    //==============================================================
        // Set Frame Poses
        frame1.SetPose(cv::Mat::eye(4,4,CV_32F));
        cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(Tcw.rowRange(0,3).col(3));
        frame2.SetPose(Tcw);

        printf("Create New KeyFrame\n");
        KeyFrame* pKFini = new KeyFrame(frame1, &map);
        KeyFrame* pKFcur = new KeyFrame(frame2, &map);

        pKFcurrent = pKFcur;

        // Insert KFs in the map
        map.AddKeyFrame(pKFini);
        map.AddKeyFrame(pKFcur);

        for (size_t i = 0; i < vnMatches12.size(); i++)
        {
            if (vnMatches12[i] < 0)
                continue;

            //Create MapPoint.
            cv::Mat worldPos(vIniP3D[i]);

            MapPoint *pMP = new MapPoint(worldPos, pKFcur, &map);

            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, vnMatches12[i]);

            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, vnMatches12[i]);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            //Fill Current Frame structure
            frame2.mvpMapPoints[vnMatches12[i]] = pMP;
            frame2.mvbOutlier[vnMatches12[i]] = false;

            //Add to Map
            map.AddMapPoint(pMP);
        }

        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        cout << "New Map created with " << map.MapPointsInMap() << " points" << endl;
    }
    else{
        printf("Intialization failed!(%d)\n", n);
        return 0;
    }

    //==============================================================
    // Tracking new frame in map
    //==============================================================
    int nmatches = orbMatcher.SearchNearby(pKFcurrent, frame3);

    printf("Matched points(%d).\n", nmatches);
    for (size_t i = 0; i < frame3.mvpMapPoints.size(); i++)
    {
        if(frame3.mvpMapPoints[i] != NULL){
            cv::KeyPoint kp = frame3.mvKeys[i];
            circle(input3, kp.pt, 2, Scalar(0, 255, 0));
        }
    }
    if(nmatches<15)
        return false;

    frame3.SetPose(frame2.mTcw);

    //==============================================================
    // Compute the pose of new frame
    //==============================================================
    CeresPoseSolver  ceressolver(&frame3);
    SamplePoseSolver samplesolver(&frame3);
    G2OPoseSolver    g2osolver(&frame3);


    auto start = std::chrono::system_clock::now(); 
    ceressolver.Solve();
    //samplesolver.Solve(3000);
    //g2osolver.Solve();

    auto end = std::chrono::system_clock::now();
    auto dur = end - start;
    auto msec = std::chrono::duration_cast<std::chrono::nanoseconds>(dur).count();
    std::cout << msec << " nanoseconds \n";


    //==============================================================
    // Draw map
    //==============================================================
    Drawer* pDrawer = new Drawer(&map);
    pDrawer->SetCameraPose(frame3.mTcw);
    std::thread th(&sample_ORB::Drawer::Run, pDrawer);
    imshow("matched keyPoints", input3);
    waitKey(0);
    th.join();
}


