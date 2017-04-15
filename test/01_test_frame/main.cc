#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ORBextractor.h"
#include "Frame.h"
#include <iostream>
const char filename_test[] = "/home/liu/workspace/sample_ORB/data/left1.png"; // 512x512 px 3 channels

using namespace cv;
using namespace sample_ORB;

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64


int main()
{
	Mat input,imGray;
	input = imread(filename_test);
    if(input.empty())
    {
        std::cout <<"Wrong image path to settings, Please check your yaml file." << std::endl;
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

    cvtColor(input, imGray, CV_RGB2GRAY);
    ORBextractor orb  =  ORBextractor(1000, 1.2, 8, 20, 7);
    Frame frame = Frame(imGray, 0, &orb, K, DistCoef);

    int t = 0;
    for (size_t col = 0; col < FRAME_GRID_COLS; col++)
    {
        for (size_t row = 0; row < FRAME_GRID_ROWS; row++)
        {
            printf("(%ld %ld):%ld\n",col,row,frame.mGrid[col][row].size());
            t += frame.mGrid[col][row].size();
            for (size_t i = 0; i < frame.mGrid[col][row].size(); i++)
            {
                cv::KeyPoint kp = frame.mvKeys[frame.mGrid[col][row][i]];
                circle(input, kp.pt, 2, Scalar((255*col/FRAME_GRID_COLS), (255*row/FRAME_GRID_ROWS), 0));
            }
        }
    }
    printf("t:%ld\n",t);
    imshow("test",input);
    waitKey(0);


}