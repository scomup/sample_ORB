
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
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawMapPoints();
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void SetDrawer(cv::Mat& CameraPose, std::list<KeyFrame*> lpLocalKeyFrames, std::vector<MapPoint*> vpLocalMapPoints);


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

};

}
#endif 
	