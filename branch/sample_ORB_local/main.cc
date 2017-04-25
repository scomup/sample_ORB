#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <boost/format.hpp> 
#include <cmath>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h>

#include "LocalMapping.h"
#include "Tracking.h"
#include <Eigen/Core>
#include <Eigen/LU>


const char strSettingsFile[] = "/home/liu/workspace/sample_ORB/config/Settings.yaml";

using namespace std;
using namespace sample_ORB;

class localSlamRunner
{

public:

    localSlamRunner(Tracking*  track):mpTracker(track),mbFirstImg(true),mTimeStamp(0),mV(0),mYawRate(0){
        

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
};

int main(int argc, char **argv)
{

Eigen::Matrix<double, 3, 3> H = Eigen::MatrixXd::Zero(3,3);

//Eigen::MatrixXd A(3,3);
H << 1.95582e+07 ,0,332556,  0,1.95582e+07,433880, 332556,433880, 1.83893e+07;
cout <<"H = " << endl << H << endl;

Eigen::Matrix<double, 3, 1> B = Eigen::MatrixXd::Zero(3,1);
B << 3660.29
,465588
,-86473.3
;
cout <<"B = " << endl << B << endl;

Eigen::VectorXd x = H.fullPivLu().solve(-B);

cout <<"x = " << endl << x << endl;
//return 0;
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
    ros::Subscriber imgsub = nodeHandler.subscribe("stereo/left/image_raw", 1, &localSlamRunner::GrabImage, &lsr);
    ros::Subscriber odmsub = nodeHandler.subscribe("Rulo/cmd_vel", 100, &localSlamRunner::GrabOdom, &lsr);

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
    mOdom[1] = mV*deltaTimeStamp;
    mOdom[2] = mYawRate*deltaTimeStamp;

    // cout<<"====================="<<endl;
    cout<<"odom:"<<mOdom<<endl;
    mpTracker->GrabImage(cv_ptr->image, mOdom ,0);
    preTimeStamp = mTimeStamp;

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