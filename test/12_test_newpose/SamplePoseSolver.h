#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/LU>
#include <istream>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include "Converter.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <mutex>
#include <cmath>
namespace sample_ORB
{

///////////////////////////////////////////////////////////////////////////////////
//# The new PoseSolver is designed to replace the g2o
//# Base on  http://europa.informatik.uni-freiburg.de/files/kuemmerle11icra.pdf
//#    .PoseSolver provides a Drawer function to help programer make sure
//#     PoseSolver worked correctly.
//#                                                        LIU.YANG
//////////////////////////////////////////////////////////////////////////////////
class SamplePoseSolver
{
  public:
    SamplePoseSolver(Frame *pFrame);
    int Solve(float maxe);
    cv::Mat Draw();

  protected:
    void Update();
    Eigen::Matrix<double, 5, 3> Jacobiani(cv::Mat point);
    Eigen::Matrix<double, 5, 1> Errori(cv::Mat point, double u, double v);
    double TotalError();
    void ReadDataFromFrame();
    double mFx;
    double mFy;
    double mCx;
    double mCy;
    double mYawc;
    double mXw;
    double mYw;
    double mDyaww;
    double mDxw;
    double mDyw;
    double mMDxw;
    double mMDyw;
    double mMDyaww;
    double mKYaw;
    double mKV;

    Frame *mpFrame;
};

SamplePoseSolver::SamplePoseSolver(Frame *pFrame) : mpFrame(pFrame)
{
    mKV = 0;
    mKYaw = 0;
}

Eigen::Matrix<double, 5, 3> SamplePoseSolver::Jacobiani(cv::Mat point)
{
    double x = point.at<float>(0);
    double y = point.at<float>(1);
    double z = point.at<float>(2);
    Eigen::Matrix<double, 5, 3> J;
    double sinY = sin(mYawc - mDyaww);
    double cosY = cos(mYawc - mDyaww);
    double X = x - mDxw - mXw;
    double Y = y - mDyw - mYw;
    double mFx_z = mFx / z;
    double mFy_z = mFy / z;
    J << mFx_z * cosY, -mFx_z * sinY, -mFx_z * (sinY * X + cosY * Y),
        mFy_z * sinY, mFy_z * cosY, mFy_z * (cosY * X - sinY * Y),
        -mKV, 0, 0,
        0, -mKV, 0,
        0, 0, -mKYaw;

    return J;
}

Eigen::Matrix<double, 5, 1> SamplePoseSolver::Errori(cv::Mat point, double u, double v)
{
    Eigen::Matrix<double, 5, 1> ei;
    double x = point.at<float>(0);
    double y = point.at<float>(1);
    double z = point.at<float>(2);
    double sinY = sin(mYawc - mDyaww);
    double cosY = cos(mYawc - mDyaww);
    double X = x - mDxw - mXw;
    double Y = y - mDyw - mYw;
    double mFx_z = mFx / z;
    double mFy_z = mFy / z;
    ei << (u - mFx_z * (cosY * X - sinY * Y) - mCx),
        (v - mFy_z * (sinY * X + cosY * Y) - mCy),
        mKV * (mMDxw - mDxw),
        mKV * (mMDyw - mDyw),
        mKYaw * (mMDyaww - mDyaww);
    return ei;
}

double SamplePoseSolver::TotalError()
{
    double tot_error = 0;
    double tot_error_proj = 0;
    double tot_error_odom = 0;
    int n = 0;

    const int N = mpFrame->N;
    for (int i = 0; i < N; i++)
    {
        MapPoint *pMP = mpFrame->mvpMapPoints[i];
        if (pMP && mpFrame->mvbOutlier[i] == false)
        {
            const cv::KeyPoint &kpUn = mpFrame->mvKeysUn[i];
            Eigen::Matrix<double, 5, 1> Ei = Errori(pMP->GetWorldPos(), kpUn.pt.x, kpUn.pt.y);

            double E2 = Ei.transpose() * Ei;
            tot_error_proj = tot_error_proj + Ei(0, 0) * Ei(0, 0) + Ei(1, 0) * Ei(1, 0);
            tot_error_odom = tot_error_odom + Ei(2, 0) * Ei(2, 0) + Ei(3, 0) * Ei(3, 0) + Ei(4, 0) * Ei(4, 0);
            tot_error = tot_error + E2;
            n++;
        }
    }
    /*
    if (tot_error_odom / n > 1)
    {
        cv::Mat bef = Draw();
        cv::imshow("bef", bef);
            cv::waitKey(1);

    }*/
    //printf("proj:%f(avg:%f) odom:%f(avg:%f)\n",tot_error_proj,tot_error_proj/n,tot_error_odom,tot_error_odom/n);
    return tot_error;
}

void SamplePoseSolver::Update()
{
    cv::Mat DRcw = Converter::computeMatrixFromAngles(0, 0, -mDyaww);
    cv::Mat Tcw = mpFrame->mTcw.clone();
    cv::Mat RcwOld = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat Rcw = RcwOld * DRcw;
    mpFrame->mRcw = Rcw;
    mpFrame->mRwc = Rcw.t();
    mpFrame->mtwc.at<float>(0) += mDxw;
    mpFrame->mtwc.at<float>(1) += mDyw;
    //tcw = -mRwc.t()*mtwc;
    mpFrame->mtcw = -Rcw * mpFrame->mtwc;
    mpFrame->mtcw.copyTo(mpFrame->mTcw.rowRange(0, 3).col(3));
    Rcw.copyTo(mpFrame->mTcw.rowRange(0, 3).colRange(0, 3));

}

cv::Mat SamplePoseSolver::Draw()
{

    cv::Mat imc = mpFrame->mpORBextractor->mvImagePyramid[0];
    cv::Mat K = mpFrame->mK;
    cv::Mat img;
    cvtColor(imc, img, CV_GRAY2RGB);

    cv::Mat Rcw = mpFrame->mTcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = mpFrame->mTcw.rowRange(0, 3).col(3);
    float pitch, roll, yaw;
    Converter::computeAnglesFromMatrix(Rcw, pitch, roll, yaw);

    const int N = mpFrame->N;
    for (int i = 0; i < N; i++)
    {
        MapPoint *pMP = mpFrame->mvpMapPoints[i];
        if (pMP)
        {
            const cv::KeyPoint &kpUn = mpFrame->mvKeysUn[i];
            cv::Mat pointw = pMP->GetWorldPos();
            cv::Mat pointc = (Rcw * pointw + tcw);
            pointc = pointc / pointc.at<float>(2);
            cv::Mat uv = K * pointc;
            float proju = uv.at<float>(0);
            float projv = uv.at<float>(1);
            cv::line(img, cv::Point(proju, projv), kpUn.pt, cv::Scalar(0, 0, 255));
            cv::circle(img, kpUn.pt, 2, cv::Scalar(0, 255, 0));
        }
    }

    return img;
}
int SamplePoseSolver::Solve(float maxe)
{
    //cv::Mat bef = Draw();
    //cv::imshow("bef",bef);
    ReadDataFromFrame();
    //Debug info:
    TotalError();
    const int N = mpFrame->N;
    double last_error = 100000000000;
    int n = 0;
    for (int loopi = 0; loopi < 10; loopi++)
    {
        n = 0;
        Eigen::Matrix<double, 3, 3> H = Eigen::MatrixXd::Zero(3, 3);
        Eigen::Matrix<double, 3, 1> B = Eigen::MatrixXd::Zero(3, 1);

        for (int i = 0; i < N; i++)
        {
            MapPoint *pMP = mpFrame->mvpMapPoints[i];
            if (pMP && mpFrame->mvbOutlier[i] == false)
            {
                const cv::KeyPoint &kpUn = mpFrame->mvKeysUn[i];
                Eigen::Matrix<double, 5, 3> Ji = Jacobiani(pMP->GetWorldPos());
                Eigen::Matrix<double, 5, 1> Ei = Errori(pMP->GetWorldPos(), kpUn.pt.x, kpUn.pt.y);
                //double E2 = Ei.transpose() * Ei;
                double E2 = Ei(0, 0) * Ei(0, 0) + Ei(1, 0) * Ei(1, 0);
                if (E2 > maxe)
                {
                    mpFrame->mvbOutlier[i] = true;
                    continue;
                }
                H = H + Ji.transpose() * Ji;
                B = B + Ji.transpose() * Ei;
                n++;

                //std::cout <<"iD = " << i << std::endl;
                //std::cout <<"H = " << std::endl << H << std::endl;
                //std::cout <<"B = " << std::endl << B << std::endl;
            }
        }

        //std::cout << "H = " << std::endl
        //          << H << std::endl;
        //std::cout << "B = " << std::endl
        //          << B << std::endl;
        Eigen::VectorXd arg = H.fullPivLu().solve(-B);
        //std::cout << "arg = " << std::endl
        //          << arg << std::endl;
        //std::cout << "============================" << std::endl;
        mDxw += arg(0, 0);
        mDyw += arg(1, 0);
        mDyaww += arg(2, 0);
        //for (int i = 0; i < N; i++)
        //{
        //    MapPoint *pMP = mpFrame->mvpMapPoints[i];
        //    if (pMP)
        //    {
        //        const cv::KeyPoint &kpUn = mpFrame->mvKeysUn[i];
        //        Eigen::Matrix<double, 2, 1> Ei = Errori(pMP->GetWorldPos(), kpUn.pt.x, kpUn.pt.y);
        //        Eigen::Matrix<double, 1, 1> E2 = Ei.transpose() * Ei;
        //        if(E2(0, 0)>10)
        //        {
        //            //printf("outlier\n");
        //            mpFrame->mvbOutlier[i] = true;
        //        }
        //        else
        //        {
        //            n++;
        //        }
        //
        //    }
        //}
        double tot_error = TotalError();
        //printf("aft:%f\n", tot_error);
        if (last_error - tot_error < 0.1)
        {
            last_error = tot_error;
            //std::cout << "The error is converge:" << std::endl;
            break;
        }
        else
        {
            last_error = tot_error;
        }
    }
    Update();
    //cv::Mat aft = Draw();
    //cv::imshow("aft",aft);
    //cv::Mat twc = mpFrame->GetCameraCenter();
    //std::cout<<twc<<std::endl;
    return n;
}

void SamplePoseSolver::ReadDataFromFrame()
{
    mFx = mpFrame->fx;
    mFy = mpFrame->fy;
    mCx = mpFrame->cx;
    mCy = mpFrame->cy;
    cv::Mat Rcw = mpFrame->mTcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat twc = mpFrame->GetCameraCenter();
    float pitch, roll, yaw;
    Converter::computeAnglesFromMatrix(Rcw, pitch, roll, yaw);
    mXw = twc.at<float>(0);
    mYw = twc.at<float>(1);
    //std::cout<<tcw<<std::endl;
    mYawc = (float)yaw;
    //mDxw    = mpFrame->mOdom[0];
    //mDyw    = mpFrame->mOdom[1];
    //mDyaww  = mpFrame->mOdom[2];
    //mMDxw   = mpFrame->mOdom[0];
    //mMDyw   = mpFrame->mOdom[1];
    //mMDyaww = mpFrame->mOdom[2];
    //If motion mode is used, set mDxw/mDyw/mDyaww and mMDxw/mMDyw/mMDyaww to zero
    mDxw = 0;
    mDyw = 0;
    mDyaww = 0;
    mMDxw = 0;
    mMDyw = 0;
    mMDyaww = 0;
}
}
