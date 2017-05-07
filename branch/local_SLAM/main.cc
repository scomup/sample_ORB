#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <string>
#include <boost/format.hpp> 
#include <cmath>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h>
#include <math.h>
#include "LocalMapping.h"
#include "Tracking.h"
#include <tf/transform_broadcaster.h>


const char strSettingsFile[] = "/home/liu/workspace/sample_ORB/config/Settings.yaml";

using namespace std;
using namespace sample_ORB;

class localSlamRunner
{

public:

    localSlamRunner(Tracking*  track):mpTracker(track),mbFirstImg(true),mTimeStamp(0),mV(0),mYawRate(0){
        
        //mTheta = atan2(CAMERA_X,CAMERA_Y);
        tf::Transform tfT;
        tfT.setIdentity();
        mTfBr.sendTransform(tf::StampedTransform(tfT,ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));  
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    //void GrabOdom(const nav_msgs::Odometry::ConstPtr &msg);
    void GrabOdom(const geometry_msgs::Twist& vel_cmd);


    Tracking*       mpTracker;
    bool            mbFirstImg;
    
    cv::Vec3f    mOdom; 
    double  mTimeStamp;
    double  mV;
    double  mYawRate;
    double  mTheta;
    tf::TransformBroadcaster mTfBr;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Mono");
    ros::start();


    Drawer*          pDrawer = new Drawer();
    LocalMapping*    pLocalMapper  = new LocalMapping();;
    Tracking*        pTracker = new Tracking(pDrawer, strSettingsFile);

    pTracker->SetLocalMapper(pLocalMapper);
    localSlamRunner  lsr(pTracker);
    std::thread      th0(&sample_ORB::LocalMapping::Run, pLocalMapper);
    std::thread      th1(&sample_ORB::Drawer::Run, pDrawer);

    

    ros::NodeHandle nodeHandler;
    ros::Subscriber imgsub = nodeHandler.subscribe("stereo/left/image_raw", 1000, &localSlamRunner::GrabImage, &lsr);
    ros::Subscriber odmsub = nodeHandler.subscribe("myRobot/cmd_vel", 1000, &localSlamRunner::GrabOdom, &lsr);

    ros::spin();

    /*
    for (int ni = 15; ni < 30; ni++)
    {
        usleep(333333);
        cv::Vec3f    mOdom; 
        mOdom[0] = 0;
        mOdom[1] = 0.11;
        mOdom[2] = 0;

        std::string str = (boost::format("/home/liu/workspace/sample_ORB/data/frame%04d.jpg") % ni).str();
        std::cout<<str<<std::endl;
        cv::Mat img = cv::imread(str);
        if (img.empty())
        {
            cerr << endl
                 << "Failed to load image at: " << str << endl;
            return 1;
        }
        pTracker->GrabImage(img, mOdom,0);
        pLocalMapper->Run();
    }*/


    ros::shutdown();
    pLocalMapper->SetFinish();
    pDrawer->SetFinish();

    th0.join();
    th1.join();

    return 0;
}

void localSlamRunner::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    static double    preTimeStamp;  
    if(mbFirstImg == true){
        preTimeStamp = msg->header.stamp.toSec();
    }
    mbFirstImg = false;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mTimeStamp = msg->header.stamp.toSec();
    double deltaTimeStamp = mTimeStamp - preTimeStamp;
    mOdom[0] = 0;
    mOdom[1] = mV;
    mOdom[2] = mYawRate;

    // cout<<"====================="<<endl;
    //cout<<"odom:"<<mOdom<<endl;
    mpTracker->GrabImage(cv_ptr->image, mOdom ,deltaTimeStamp);
    preTimeStamp = mTimeStamp;
    cv::Mat Tcw = mpTracker->mCurrentFrame.mTcw;

    if (Tcw.empty() != true)
    {
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
        tf::Matrix3x3 M(Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
                        Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
                        Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2));
        tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));
        tf::Transform tfTcw(M, V);
        mTfBr.sendTransform(tf::StampedTransform(tfTcw, ros::Time::now(), "ORB_SLAM/World", "ORB_SLAM/Camera"));
    }
}
/*
void localSlamRunner::GrabOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(mbFirstImg)
        return;
    double roll, pitch, yaw;
    tf::Quaternion q(msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    mOdom[0] = -msg->pose.pose.position.y;
    mOdom[1] = msg->pose.pose.position.x;
    mOdom[2] = yaw;
}*/

void localSlamRunner::GrabOdom(const geometry_msgs::Twist& vel_cmd)
{
    mV = vel_cmd.linear.x;
    mYawRate = vel_cmd.angular.z;

}