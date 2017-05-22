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

// A CostFunction implementing analytically derivatives for the
// function f(x) = 10 - x.
class ReprojectionError2 : public ceres::SizedCostFunction<2, 6>
{
  public:
    ReprojectionError2(cv::KeyPoint kp, cv::Mat mp, cv::Mat K) : u(kp.pt.x),
                                                                 v(kp.pt.y),
                                                                 x(mp.at<float>(0)),
                                                                 y(mp.at<float>(1)),
                                                                 z(mp.at<float>(2)),
                                                                 fx(K.at<float>(0, 0)),
                                                                 fy(K.at<float>(1, 1)),
                                                                 cx(K.at<float>(0, 2)),
                                                                 cy(K.at<float>(1, 2)) {}

    virtual ~ReprojectionError2(){}

    virtual bool Evaluate(double const *const *camera,
                          double *residuals,
                          double **jacobians) const
    {
        double p[3];
        {
            double sx = sin(camera[0][0]);
            double cx = cos(camera[0][0]);
            double sy = sin(camera[0][1]);
            double cy = cos(camera[0][1]);
            double sz = sin(camera[0][2]);
            double cz = cos(camera[0][2]);
            p[0] = (cy * cz - sx * sy * sz) * x + (-cx * sz) * y + (sy * cz + sx * cy * sz) * z + camera[0][3];
            p[1] = (cy * sz + sx * sy * cz) * x + (cx * cz) * y + (sy * sz - sx * cy * cz) * z + camera[0][4];
            p[2] = -cx * sy * x + sx * y + cx * cy * z + camera[0][5];
        }

        double predicted_x, predicted_y;
        {
            predicted_x = fx * (p[0] / p[2]) + cx;
            predicted_y = fy * (p[1] / p[2]) + cy;
        }

        residuals[0] = predicted_x - u;
        residuals[1] = predicted_y - v;

        // Compute the Jacobian if asked for.
        {
            if (!jacobians)
                return true;
            double x = p[0];
            double y = p[1];
            double z = p[2];
            double z_2 = z * z;

            double *jacobian = jacobians[0];
            if (!jacobian)
                return true;
            jacobian[0]  = -x * y / z_2 * fx;
            jacobian[1]  = (1 + (x * x / z_2)) * fx;
            jacobian[2]  = -y / z * fx;
            jacobian[3]  = 1. / z * fx;
            jacobian[4]  = 0;
            jacobian[5]  = -x / z_2 * fx;
            jacobian[6]  = -(1 + y * y / z_2) * fy;
            jacobian[7]  = x * y / z_2 * fy;
            jacobian[8]  = x / z * fy;
            jacobian[9]  = 0;
            jacobian[10] = 1. / z * fy;
            jacobian[11] = -y / z_2 * fy;

        }
        return true;

    }
    double u;
    double v;
    double x;
    double y;
    double z;
    double fx;
    double fy;
    double cx;
    double cy;

};

struct ReprojectionError
{

    ReprojectionError(cv::KeyPoint kp, cv::Mat mp, cv::Mat K) : u(kp.pt.x),
                                                                v(kp.pt.y),
                                                                x(mp.at<float>(0)),
                                                                y(mp.at<float>(1)),
                                                                z(mp.at<float>(2)),
                                                                fx(K.at<float>(0, 0)),
                                                                fy(K.at<float>(1, 1)),
                                                                cx(K.at<float>(0, 2)),
                                                                cy(K.at<float>(1, 2))
    {
    }

    template <typename T>
    bool operator()(const T *const camera, T *residuals) const
    {
        T p[3];
        {
            T sx = sin(camera[0]);
            T cx = cos(camera[0]);
            T sy = sin(camera[1]);
            T cy = cos(camera[1]);
            T sz = sin(camera[2]);
            T cz = cos(camera[2]);
            p[0] = (cy * cz - sx * sy * sz) * T(x) + (-cx * sz) * T(y) + (sy * cz + sx * cy * sz) * T(z) + camera[3];
            p[1] = (cy * sz + sx * sy * cz) * T(x) + (cx * cz) * T(y) + (sy * sz - sx * cy * cz) * T(z) + camera[4];
            p[2] = -cx * sy * T(x) + sx * T(y) + cx * cy * T(z) + camera[5];
        }

        T predicted_x, predicted_y;
        {

            predicted_x = fx * (p[0] / p[2]) + cx;
            predicted_y = fy * (p[1] / p[2]) + cy;
        }

        residuals[0] = predicted_x - T(u);
        residuals[1] = predicted_y - T(v);

        return true;
    }
    double u;
    double v;
    double x;
    double y;
    double z;
    double fx;
    double fy;
    double cx;
    double cy;
};

///////////////////////////////////////////////////////////////////////////////////
//# The new PoseSolver is designed to replace the g2o
//# Base on  http://europa.informatik.uni-freiburg.de/files/kuemmerle11icra.pdf
//#    .PoseSolver provides a Drawer function to help programer make sure
//#     PoseSolver worked correctly.
//#                                                        LIU.YANG
//////////////////////////////////////////////////////////////////////////////////
class CeresPoseSolver
{
  public:
    CeresPoseSolver(Frame *pFrame);
    int Solve();
    cv::Mat Draw();

  protected:
    void Update(double *camera);
    Frame *mpFrame;
};

CeresPoseSolver::CeresPoseSolver(Frame *pFrame) : mpFrame(pFrame)
{
}

void CeresPoseSolver::Update(double *camera)
{
    cv::Mat Rcw = Converter::computeMatrixFromAngles(camera[0], camera[1], camera[2]);
    cv::Mat tcw = cv::Mat(3, 1, CV_32FC1);
    tcw.at<float>(0) = float(camera[3]);
    tcw.at<float>(1) = float(camera[4]);
    tcw.at<float>(2) = float(camera[5]);
    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32FC1);
    Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
    tcw.copyTo(Tcw.rowRange(0, 3).col(3));
    mpFrame->SetPose(Tcw);
}

cv::Mat CeresPoseSolver::Draw()
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

int CeresPoseSolver::Solve()
{
    //cv::Mat bef = Draw();
    //cv::imshow("bef", bef);
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
    cv::Mat K = mpFrame->mK;

    //printf("(%f %f %f),(%f %f %f)\n",camera[0],camera[1],camera[2],camera[3],camera[4],camera[5]);

    ceres::Problem problem;

    for (int i = 0; i < N; i++)
    {
        MapPoint *pMP = mpFrame->mvpMapPoints[i];
        if (pMP && mpFrame->mvbOutlier[i] == false)
        {
            const cv::KeyPoint &kpUn = mpFrame->mvKeysUn[i];
            cv::Mat mp = pMP->GetWorldPos();

            //ceres::CostFunction* cost_function = new ReprojectionError2(kpUn, mp, K);
            
            ReprojectionError *repError = new ReprojectionError(kpUn, mp, K);
            ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(repError);

            problem.AddResidualBlock(cost_function, NULL, camera);
        }
    }

    ceres::Solver::Options options; //
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //printf("(%f %f %f),(%f %f %f)\n", camera[0], camera[1], camera[2], camera[3], camera[4], camera[5]);
    Update(camera);
    //cv::Mat aft = Draw();
    //cv::imshow("aft", aft);
    //cv::waitKey(0);

    //std::cout << summary.BriefReport() << std::endl;

    //printf("(%f %f %f),(%f %f %f)\n", camera[0], camera[1], camera[2], camera[3], camera[4], camera[5]);

    return 0;
}
}
