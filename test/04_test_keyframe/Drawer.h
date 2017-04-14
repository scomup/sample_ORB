
#ifndef DRAWR_H
#define DRAWR_H

#include <mutex>
#include <opencv2/core/core.hpp>

class PointDrawer
{
public:
    PointDrawer();

    void Run();
    void update(std::vector<cv::Point3f> p3d);

private:
    std::vector<cv::Point3f> mP3ds;
    std::mutex mTx_;
};


#endif 
	
