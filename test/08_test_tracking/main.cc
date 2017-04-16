#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <boost/format.hpp> 
#include <iostream>

#include "Tracking.h"

const char strSettingsFile[] = "/home/liu/workspace/sample_ORB/config/Settings.yaml";

using namespace cv;
using namespace sample_ORB;
int main()
{
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }
    
    //Create KeyFrame Database
    //mpKeyFrameDatabase = new KeyFrameDatabase();

    //Create the Map
    Map* mpMap = new Map();
    Drawer* pDrawer = new Drawer(mpMap);
    std::thread th(&sample_ORB::Drawer::Run, pDrawer);
    Tracking* mpTracker = new Tracking(pDrawer, mpMap, strSettingsFile);
    cv::Mat img;
    for (int ni = 15; ni < 30; ni++)
    {
        usleep(333333);
        std::string str = (boost::format("/home/liu/workspace/sample_ORB/data/frame%04d.jpg") % ni).str();
        std::cout<<str<<std::endl;
        img = cv::imread(str);
        if (img.empty())
        {
            cerr << endl
                 << "Failed to load image at: " << str << endl;
            return 1;
        }
        mpTracker->GrabImage(img, 0);
    }
    th.join();
}


