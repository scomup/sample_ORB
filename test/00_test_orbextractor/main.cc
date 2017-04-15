#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ORBextractor.h"

#include <iostream>
const char filename_test[] = "/home/liu/workspace/sample_ORB/data/left1.png"; // 512x512 px 3 channels

using namespace cv;
using namespace sample_ORB;
int main()
{
	Mat input,output;
	input = imread(filename_test);
    if(input.empty())
    {
        std::cout <<"Wrong image path to settings, Please check your yaml file." << std::endl;
        return 1;
    }

    cvtColor(input, input, CV_RGB2GRAY);
    ORBextractor orb  =  ORBextractor(1000, 1.2, 8, 20, 7);

    std::vector<cv::KeyPoint> keys;
    cv::Mat descriptors;

    orb(input,cv::Mat(),keys,descriptors);
    imwrite("DrawKeyPoints.png",input);
    drawKeypoints(input, keys, input);
    imshow("DrawKeyPoints",input);
    printf("The number of KeyPoints:%ld\n",keys.size());

    waitKey(0);

}