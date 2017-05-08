#include "Drawer.h"
#include <pangolin/pangolin.h>

#include <mutex>
#include <limits>


namespace sample_ORB
{
    

Drawer::Drawer():
mKeyFrameSize(0.1),
mKeyFrameLineWidth(2),
mGraphLineWidth(1.8),
mPointSize(4),
mCameraSize(mKeyFrameSize * 2),
mCameraLineWidth(mKeyFrameLineWidth * 2),
mbFinished(false){}

void Drawer::Run()
{

     pangolin::CreateWindowAndBind("Main",1024,768);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

   // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
        pangolin::ModelViewLookAt(0,-0.7,-1.8, 0.5,0.5,0,  pangolin::AxisNegY)
    );
    pangolin::Handler3D handler(s_cam);
    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f).SetHandler(&handler);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix TwcOdom;
    TwcOdom.SetIdentity();
    cv::Scalar color1 = cv::Scalar(0,255,0);
    cv::Scalar color2 = cv::Scalar(0,255,255);


    while(1)
    {        
        std::unique_lock<std::mutex> lk(mTx_);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f,0.0f,0.0f,0.0f);

        GetCurrentOpenGLCameraMatrix(Twc, mCameraPose);
        GetCurrentOpenGLCameraMatrix(TwcOdom, mCameraPoseOdom);
        DrawKeyFrames(true,true);
        DrawMapPoints();
        DrawCurrentCamera(Twc, color1);
        DrawCurrentCamera(TwcOdom, color2);
        DrawPath();
        pangolin::FinishFrame();

        lk.unlock();
        usleep(3000);

        if (isFinished())
        {
            break;
        }
    }
}

void Drawer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
}

bool Drawer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Drawer::DrawPath()
{
    glLineWidth(3);
    glColor3f(0,1,0);
    glBegin(GL_LINE_STRIP);
    for (size_t i = 0, iend = mvpPathPoints.size(); i < iend; i++)
    {
        cv::Vec3f point = mvpPathPoints[i];
        glVertex3f(point[0],point[1],point[2]);
    }
    glEnd();
}

void Drawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc, cv::Scalar color)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    float r = color[0]/255.0;
    float g = color[1]/255.0;
    float b = color[2]/255.0;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(r,g,b);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void Drawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const std::vector<KeyFrame*> vpKFs(mlpLocalKeyFrames.begin(), mlpLocalKeyFrames.end());

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

/*
    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const std::vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(std::vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }
        }

        glEnd();
    }*/
}

void Drawer::DrawMapPoints()
{
    const std::vector<MapPoint*> &vpMPs = mvpLocalMapPoints;
    //const std::vector<MapPoint*> &vpRefMPs = mvpLocalMapPoints;

    //std::set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->mnLastFrameSeen != mnCurId)
            glColor3f(1.0,0.0,0.0);
        else
            glColor3f(0.0,1.0,0.0);
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();
}
void Drawer::SetDrawer(cv::Mat& CameraPose, cv::Mat& CameraPoseOdom,std::list<KeyFrame*> lpLocalKeyFrames, std::vector<MapPoint*> vpLocalMapPoints, long unsigned int nCurId){
    mCameraPose = CameraPose;
    mCameraPoseOdom = CameraPoseOdom;
    mvpLocalMapPoints = vpLocalMapPoints;
    mlpLocalKeyFrames = lpLocalKeyFrames;
    mnCurId = nCurId;
    cv::Vec3f point;
    cv::Mat Rcw = CameraPose.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = CameraPose.rowRange(0,3).col(3);
    cv::Mat twc = -Rcw.t()*tcw;
    point[0] = twc.at<float>(0);
    point[1] = twc.at<float>(1);
    point[2] = twc.at<float>(2);
    mvpPathPoints.push_back(point);

}

void Drawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, cv::Mat CameraPose)
{
    if(!CameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            std::unique_lock<std::mutex> lock(mMutexCamera);
            Rwc = CameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*CameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}




}

