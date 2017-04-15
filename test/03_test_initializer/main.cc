#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ORBextractor.h"
#include "Frame.h"
#include "ORBmatcher.h"
#include "Initializer.h"
#include "Drawer.h"

#include <iostream>
const char filename_test1[] = "/home/liu/workspace/sample_ORB/data/frame0015.jpg";
const char filename_test2[] = "/home/liu/workspace/sample_ORB/data/frame0018.jpg";

using namespace cv;
using namespace sample_ORB;
int main()
{
    Mat input1, input2;
    Mat imGray1, imGray2;
    input1 = imread(filename_test1);
    input2 = imread(filename_test2);
    input1 = imread(filename_test1);
    input2 = imread(filename_test2);
    if (input1.empty() || input2.empty())
    {
        std::cout << "Wrong image path to settings, Please check your yaml file." << std::endl;
        return 1;
    }

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = 320;
    K.at<float>(1, 1) = 320;
    K.at<float>(0, 2) = 320.5;
    K.at<float>(1, 2) = 240.5;

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = 0;
    DistCoef.at<float>(1) = 0;
    DistCoef.at<float>(2) = 0;
    DistCoef.at<float>(3) = 0;

    cvtColor(input1, imGray1, CV_RGB2GRAY);
    cvtColor(input2, imGray2, CV_RGB2GRAY);
    ORBextractor orb = ORBextractor(2000, 1.2, 8, 20, 7);
    Frame frame1 = Frame(imGray1, 0, &orb, K, DistCoef);
    Frame frame2 = Frame(imGray2, 0, &orb, K, DistCoef);
    ORBmatcher orbMatcher = ORBmatcher(0.9, true);
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

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
    std::vector<cv::Point3f> vIniP3D;

    Initializer initializer =  Initializer(frame1,1.0,200);
    if(initializer.Initialize(frame2, vnMatches12, Rcw, tcw, vIniP3D, vbTriangulated)){
        printf("Intialized!(%d)\n", n);
        PointDrawer drawer;
        drawer.update(vIniP3D);
        drawer.Run();
    }
    else{
        printf("Intialization failed!(%d)\n", n);
    }
}