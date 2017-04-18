
#ifndef DRAWR_H
#define DRAWR_H

#include <mutex>
#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>

#include "Map.h"
namespace sample_ORB
{

class Drawer
{
public:
    Drawer(Map *pMap);

    void Run();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawMapPoints();
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void SetCameraPose(cv::Mat& CameraPose);


private:
    Map* mpMap; 
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
	
