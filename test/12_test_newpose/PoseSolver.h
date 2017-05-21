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
struct ReprojectionError
{

    ReprojectionError(double* observed, double*point, double *K)
        : observed(observed), point(point), K(K) {}

    template <typename T>
    bool operator()(const T *const camera, T *residuals) const
    {
        //Project the point to camera coordinates.
        T p[3];
        {
            //auto pitch, roll, yaw;
            
            //T sx = sin(camera[0]);
            //T cx = cos(camera[0]);
            //T sy = sin(camera[1]);
            //T cy = cos(camera[1]);
            //T sz = sin(camera[2]);
            //T cz = cos(camera[2]);
            T sx = T(0.0);
            T cx = T(1.0);
            T sy = T(0.0);
            T cy = T(1.0);
            T sz = T(0.0);
            T cz = T(1.0);
/*
            cv::Mat R = cv::Mat::zeros(cv::Size(3,3), CV_32F);
            R.row(0).col(0) = cy*cz - sx*sy*sz;
            R.row(0).col(1) = -cx*sz;
            R.row(0).col(2) = sy*cz + sx*cy*sz;
            R.row(1).col(0) = cy*sz + sx*sy*cz;
            R.row(1).col(1) = cx*cz;
            R.row(1).col(2) = sy*sz - sx*cy*cz;
            R.row(2).col(0) = - cx*sy;
            R.row(2).col(1) = sx;
            R.row(2).col(2) = cx*cy;
*/
            p[0] = (cy*cz - sx*sy*sz) * T(point[0]) + (-cx*sz) * T(point[1]) + (sy*cz + sx*cy*sz) * T(point[2]) ;
            p[1] = (cy*sz + sx*sy*cz)* T(point[0]) + (cx*cz) * T(point[1]) + (sy*sz - sx*cy*cz) * T(point[2]) -T(1.8);
            p[2] =     - cx*sy * T(point[0]) +                  sx * T(point[1])                  + cx*cy * T(point[2]);


            //p[0] = (cy*cz - sx*sy*sz) * T(point[0]) + (-cx*sz) * T(point[1]) + (sy*cz + sx*cy*sz) * T(point[2]) + camera[3];
            //p[1] = (cy*sz + sx*sy*cz)* T(point[0]) + (cx*cz) * T(point[1]) + (sy*sz - sx*cy*cz) * T(point[2]) + camera[4];
            //p[2] =     - cx*sy * T(point[0]) +                  sx * T(point[1])                  + cx*cy * T(point[2]) + camera[5];


            //float 
            //cv::Mat Rcw = Converter::computeMatrixFromAngles(float(camera[0]), float(camera[1]), float(camera[2]));
            //cv::Mat tcw = cv::Mat(3, 1, CV_32FC1);
            //cv::Mat p_mat   = cv::Mat(3, 1, CV_32FC1);
            //p_mat.at<float>(0) =  float(point[0]);
            //p_mat.at<float>(1) =  float(point[1]);
            //p_mat.at<float>(2) =  float(point[2]);
            //tcw.at<float>(0) =  float(camera[3]);
            //tcw.at<float>(1) =  float(camera[4]);
            //tcw.at<float>(2) =  float(camera[5]);
            //cv::Mat k = cv::Mat::eye(3, 3, CV_32FC1);
            //k.at<float>(0,0) =  float(K[0]);
            //k.at<float>(1,1) =  float(K[1]);
            //k.at<float>(0,2) =  float(K[2]);
            //k.at<float>(1,2) =  float(K[3]); 
 //
//
            //p_mat = k*(Rcw*p_mat+tcw);
            //p[0] = tcw.at<float>(0);
            //p[1] = tcw.at<float>(1);
            //p[2] = tcw.at<float>(2);
           
               
            /*
            T sx = sin(camera[0]);
            T cx = cos(camera[0]);
            T sy = sin(camera[1]);
            T cy = cos(camera[1]);
            T sz = sin(camera[2]);
            T cz = cos(camera[2]);

            p[0] = cy * cz * T(point[0]) + (cz * sx * sy - cx * sz) * T(point[1]) + (sx * sz + cx * cz * sy) * T(point[2]) + camera[3];
            p[1] = cy * sz * T(point[0]) + (cx * cz + sx * sy * sz) * T(point[1]) + (cx * sy * sz - cz * sx) * T(point[2]) + camera[4];
            p[2] =     -sy * T(point[0]) +                  cy * sx * T(point[1])                  + cx * cy * T(point[2]) + camera[5];
            */
        }

        //Project the point to image coordinates.
        T predicted_x, predicted_y;
        {
            T fx = T(K[0]);
            T fy = T(K[1]);
            T cx = T(K[2]);
            T cy = T(K[3]);

            predicted_x = fx * (p[0] / p[2]) + cx;
            predicted_y = fy * (p[1] / p[2]) + cy;
        }

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed[0]);
        residuals[1] = predicted_y - T(observed[1]);
        //residuals[0] = predicted_x;
        //residuals[1] = predicted_y;


        return true;
    }

    static ceres::CostFunction *Create(double *observed, double *point, double *K)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(
            new ReprojectionError(observed, point, K)));
    }

    double* observed;
    double* point;
    double* K;
};

///////////////////////////////////////////////////////////////////////////////////
//# The new PoseSolver is designed to replace the g2o
//# Base on  http://europa.informatik.uni-freiburg.de/files/kuemmerle11icra.pdf
//#    .PoseSolver provides a Drawer function to help programer make sure
//#     PoseSolver worked correctly.
//#                                                        LIU.YANG
//////////////////////////////////////////////////////////////////////////////////
class PoseSolver
{
  public:
    PoseSolver(Frame *pFrame);
    int Solve();
    cv::Mat Draw();

  protected:
    void Update(double* camera);
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
    Frame *mpFrame;
};

PoseSolver::PoseSolver(Frame *pFrame) : mpFrame(pFrame)
{
    ReadDataFromFrame();
}

void PoseSolver::Update(double* camera)
{
    cv::Mat Rcw = Converter::computeMatrixFromAngles(camera[0], camera[1], camera[2]);
    cv::Mat tcw = cv::Mat(3, 1, CV_32FC1);
    tcw.at<float>(0) =  float(camera[3]);
    tcw.at<float>(1) =  float(camera[4]);
    tcw.at<float>(2) =  float(camera[5]);

    cv::Mat Tcw = cv::Mat::eye(4,4,CV_32FC1);
    Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
    tcw.copyTo(Tcw.rowRange(0,3).col(3));
    mpFrame->SetPose(Tcw);

}

cv::Mat PoseSolver::Draw()
{
    

    cv::Mat imc = mpFrame->mpORBextractor->mvImagePyramid[0];
    //cv::Mat K = mpFrame->mK;
    cv::Mat img;
    cvtColor(imc, img, CV_GRAY2RGB);

    cv::Mat Tcw = mpFrame->mTcw;

    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    
    double camera[6];
    float pitch, roll, yaw;
    Converter::computeAnglesFromMatrix(Rcw, pitch, roll, yaw);
    cv::Mat NewR = Converter::computeMatrixFromAngles(pitch, roll, yaw);
    camera[0] = pitch;
    camera[1] = roll;
    camera[2] = yaw;
    camera[3] = tcw.at<float>(0);
    camera[4] = tcw.at<float>(1);
    camera[5] = tcw.at<float>(2);
    printf("(%f %f %f),(%f %f %f)\n",camera[0],camera[1],camera[2],camera[3],camera[4],camera[5]);

    double K[4] = {mFx, mFy, mCx, mCy};


    const int N = mpFrame->N;
    for (int i = 0; i < N; i++)
    {
        MapPoint *pMP = mpFrame->mvpMapPoints[i];
        if (pMP)
        {
            const cv::KeyPoint &kpUn = mpFrame->mvKeysUn[i];
            double observed[2] = {kpUn.pt.x, kpUn.pt.y};

            cv::Mat mp = pMP->GetWorldPos();
            //std::cout<<Rcw<<std::endl;
            //std::cout<<NewR<<std::endl;

            double point[3] = { double(mp.at<float>(0)), 
                                double(mp.at<float>(1)), 
                                double(mp.at<float>(2))};

            ReprojectionError repError(observed, point, K);
            double uv[2];
            repError(camera, uv);
            printf("%f %f\n",uv[0], uv[1]);
            cv::line(img, cv::Point(uv[0], uv[1]), kpUn.pt, cv::Scalar(0, 0, 255));
            cv::circle(img, kpUn.pt, 2, cv::Scalar(0, 255, 0));
        }
    }

    return img;
    
    cv::Mat a;
    return a;
}

int PoseSolver::Solve()
{
    cv::Mat bef = Draw();
    //cv::imshow("bef", bef);
    //cv::waitKey(0);
//
    const int N = mpFrame->N;

    cv::Mat Rcw = mpFrame->mTcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = mpFrame->mTcw.rowRange(0, 3).col(3);
    double camera[6];

    float pitch, roll, yaw;
    Converter::computeAnglesFromMatrix(Rcw, pitch, roll, yaw);

    camera[0] = pitch;
    camera[1] = roll;
    camera[2] = yaw;

    camera[3] = tcw.at<float>(0);
    camera[4] = tcw.at<float>(1);
    camera[5] = tcw.at<float>(2);

    printf("(%f %f %f),(%f %f %f)\n",camera[0],camera[1],camera[2],camera[3],camera[4],camera[5]);
    double K[4] = {mFx, mFy, mCx, mCy};

    ceres::Problem problem;

{
    for (int i = 0; i < N; i++)
    {
        MapPoint *pMP = mpFrame->mvpMapPoints[i];
        if (pMP && mpFrame->mvbOutlier[i] == false)
        {
            const cv::KeyPoint &kpUn = mpFrame->mvKeysUn[i];
            double observed[2] = {kpUn.pt.x, kpUn.pt.y};
            cv::Mat mp = pMP->GetWorldPos();
            double point[3] = {mp.at<float>(0), mp.at<float>(1), mp.at<float>(2)};
            
            ReprojectionError* repError = new ReprojectionError(observed, point, K);

            ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(repError);
                        double uv[2];
            (*repError)(camera,uv);
            printf("error:= %f\n",uv[0]*uv[0] + uv[1]*uv[1]);
            problem.AddResidualBlock(cost_function, NULL, camera);
            //break;
        }
    }

  ceres::Solver::Options options;//最適化のオプション設定用構造体
  options.linear_solver_type=ceres::DENSE_QR;
  options.minimizer_progress_to_stdout=true;//最適化の結果を標準出力に表示する。
  ceres::Solver::Summary summary;//最適化の結果を格納するよう構造体
  ceres::Solve(options,&problem,&summary);//最適化の実行
    std::cout<<summary.BriefReport()<<std::endl;

}

  //結果の表示
    printf("(%f %f %f),(%f %f %f)\n",camera[0],camera[1],camera[2],camera[3],camera[4],camera[5]);
    Update(camera);
    cv::Mat aft = Draw();

    //Update(camera);
    //std::cout << summary.FullReport() << "\n";

    return 0;
}

void PoseSolver::ReadDataFromFrame()
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
    mDyaww = 0;
    mDxw = 0;
    mDyw = 0;
}
}
