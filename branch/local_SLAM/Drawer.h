
#ifndef DRAWR_H
#define DRAWR_H

#include <mutex>
#include <opencv2/core/core.hpp>
#include "Tracking.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <list>

namespace sample_ORB
{
class Tracking;
class KeyFrame;

class Drawer

{
public:
    Drawer();

    void Run();
    void SetFinish();
    bool isFinished();
    void SetDrawer(cv::Mat& CameraPose, cv::Mat& CameraPoseOdom,std::list<KeyFrame*> lpLocalKeyFrames, std::vector<MapPoint*> vpLocalMapPoints, long unsigned int nCurId);


protected:

    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawMapPoints();
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc, cv::Scalar color);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, cv::Mat CameraPose);

private:
    std::list<KeyFrame*> mlpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    std::mutex mTx_;
    std::mutex mMutexCamera;
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    cv::Mat mCameraPose;
    cv::Mat mCameraPoseOdom;
    bool mbFinished;
    std::mutex mMutexFinish;
    long unsigned int mnCurId;

};

}
#endif 
	
