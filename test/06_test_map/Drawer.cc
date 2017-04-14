#include "Drawer.h"
#include <pangolin/pangolin.h>

#include <mutex>

#include <limits>

PointDrawer::PointDrawer()
{
}

void PointDrawer::Run()
{

     pangolin::CreateWindowAndBind("Main",1024,768);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

   // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024,768,600,600,512,389,0.1,1000),
        pangolin::ModelViewLookAt(0.5,0.5,-2, 0.5,0.5,0,  pangolin::AxisNegY)
    );
    pangolin::Handler3D handler(s_cam);
    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f).SetHandler(&handler);

    while(1)
    {
        
        std::unique_lock<std::mutex> lk(mTx_);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glPointSize(3);

        glBegin(GL_POINTS);
        glColor3f(0.0,1.0,0.0);
        for(size_t i=0; i<mP3ds.size();i++)
        {
            glVertex3f(mP3ds[i].x,mP3ds[i].y,mP3ds[i].z);
        }
        glEnd();

        pangolin::FinishFrame();

        lk.unlock();
        usleep(3000);
        
    }
}


void PointDrawer::update(std::vector<cv::Point3f> p3ds){
    mP3ds = p3ds;
}