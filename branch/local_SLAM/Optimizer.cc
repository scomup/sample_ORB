/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#define USE_POSESOLVER


#include "Optimizer.h"

#include "../../Thirdparty/g2o/g2o/core/block_solver.h"
#include "../../Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "../../Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "../../Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "../../Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "../../Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "../../Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/LU>
#include "Converter.h"

#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

namespace sample_ORB
{

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
    int Solve(float maxe);
    cv::Mat  Draw();


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

PoseSolver::PoseSolver(Frame *pFrame) : mpFrame(pFrame)
{
     mKV = 50;
     mKYaw = 10;       
}

Eigen::Matrix<double, 5, 3> PoseSolver::Jacobiani(cv::Mat point)
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
    J << mFx_z*cosY, -mFx_z*sinY,   -mFx_z*(sinY * X + cosY * Y),
         mFy_z*sinY,  mFy_z*cosY,    mFy_z*(cosY * X - sinY * Y),
         -mKV,        0,             0,
         0,           -mKV,          0,
         0,           0,            -mKYaw;

    return J;
}

Eigen::Matrix<double, 5, 1> PoseSolver::Errori(cv::Mat point, double u, double v)
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
          mKV*(mMDxw    - mDxw),
          mKV*(mMDyw    - mDyw),
          mKYaw*(mMDyaww  - mDyaww);
    return ei;
}

double PoseSolver::TotalError()
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
            tot_error_proj = tot_error_proj + Ei(0,0) * Ei(0,0) + Ei(1,0) * Ei(1,0);
            tot_error_odom = tot_error_odom + Ei(2,0) * Ei(2,0) + Ei(3,0) * Ei(3,0) + Ei(4,0) * Ei(4,0);
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

void PoseSolver::Update()
{
    cv::Mat DRcw = Converter::computeMatrixFromAngles(0, 0, -mDyaww);
    cv::Mat Tcw = mpFrame-> mTcw.clone();
    cv::Mat RcwOld = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat Rcw = RcwOld * DRcw;
    mpFrame->mRcw = Rcw;
    mpFrame->mRwc = Rcw.t();
    mpFrame->mtwc.at<float>(0) +=  mDxw;
    mpFrame->mtwc.at<float>(1) +=  mDyw;
    //tcw = -mRwc.t()*mtwc;
    mpFrame->mtcw = -Rcw*mpFrame->mtwc;
    mpFrame->mtcw.copyTo(mpFrame->mTcw.rowRange(0,3).col(3));
    Rcw.copyTo(mpFrame->mTcw.rowRange(0,3).colRange(0,3));
    mMDxw = mpFrame->mOdom[0] = mMDxw    - mDxw;
    mMDyw = mpFrame->mOdom[1] = mMDyw    - mDyw;
    mMDyaww = mpFrame->mOdom[2] = mMDyaww  - mDyaww;

    debug_printf("opt:dx:%f dy:%f dyaw%f\n",mDxw,mDyw,mDyaww);


}

cv::Mat PoseSolver::Draw()
{
    
    cv::Mat imc = mpFrame->mpORBextractor->mvImagePyramid[0];
    cv::Mat K = mpFrame->mK;
    cv::Mat img;
    cvtColor(imc, img, CV_GRAY2RGB);

    cv::Mat Rcw = mpFrame->mTcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = mpFrame->mTcw.rowRange(0,3).col(3);
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
            cv::Mat uv = K*pointc;
            float proju = uv.at<float>(0);
            float projv = uv.at<float>(1);
            cv::line(img, cv::Point(proju,projv), kpUn.pt, cv::Scalar(0, 0, 255));
            cv::circle(img, kpUn.pt, 2, cv::Scalar(0, 255, 0));
        }
    }

    return img;
}
int PoseSolver::Solve(float maxe)
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
                double E2 = Ei(0,0) * Ei(0,0) + Ei(1,0) * Ei(1,0);
                if(E2 > maxe){
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
        if ( last_error - tot_error < 0.1)
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

void PoseSolver::ReadDataFromFrame()
{
    mFx = mpFrame->fx;
    mFy = mpFrame->fy;
    mCx = mpFrame->cx;
    mCy = mpFrame->cy;
    cv::Mat Rcw = mpFrame->mTcw.rowRange(0,3).colRange(0,3);
    cv::Mat twc = mpFrame->GetCameraCenter();
    float pitch, roll, yaw;
    Converter::computeAnglesFromMatrix(Rcw, pitch, roll, yaw);
    mXw = twc.at<float>(0);
    mYw = twc.at<float>(1);
    //std::cout<<tcw<<std::endl;
    mYawc   = (float)yaw;
    //mDxw    = mpFrame->mOdom[0];
    //mDyw    = mpFrame->mOdom[1];
    //mDyaww  = mpFrame->mOdom[2];
    //mMDxw   = mpFrame->mOdom[0];
    //mMDyw   = mpFrame->mOdom[1];
    //mMDyaww = mpFrame->mOdom[2];
    //If motion mode is used, set mDxw/mDyw/mDyaww and mMDxw/mMDyw/mMDyaww to zero
    mDxw    = 0;
    mDyw    = 0;
    mDyaww  = 0;
    mMDxw   = 0;
    mMDyw   = 0;
    mMDyaww = 0;
}
}

namespace sample_ORB
{
using namespace std;
#ifdef USE_POSESOLVER

int Optimizer::PoseOptimization(Frame *pFrame)
{
    int n;
    PoseSolver solver(pFrame);

    //printf("Solve300\n");
    n = solver.Solve(80);
    //printf("Solve100\n");
    n = solver.Solve(30);
    return n;
}

#else

int Optimizer::PoseOptimization(Frame *pFrame)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences = 0;

    // Set Frame vertex
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N; i++)
        {
            MapPoint *pMP = pFrame->mvpMapPoints[i];
            if (pMP)
            {
                // Monocular observation

                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double, 2, 1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
        }
    }

    if (nInitialCorrespondences < 3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
    const int its[4] = {10, 10, 10, 10};

    int nBad = 0;
    for (size_t it = 0; it < 4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it])
            {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                e->setLevel(0);
                pFrame->mvbOutlier[idx] = false;
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        if (optimizer.edges().size() < 10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);
    cv::Mat twc = pFrame->GetCameraCenter();
    return nInitialCorrespondences - nBad;
}
#endif



void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, list<KeyFrame *> lLocalKeyFrames, bool *pbStopFlag)
{
    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
                if (pMP->mnBALocalForKF != pKF->mnId)
                {
                    lLocalMapPoints.push_back(pMP);
                    pMP->mnBALocalForKF = pKF->mnId;
                }
        }
    }

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame *> lFixedCameras;

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    map<KeyFrame *, int> KFcounter;
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        if (KFcounter.count(pKFi))
            continue;
        KFcounter[pKFi]++;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);

    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame *, size_t> observations = pMP->GetObservations();

        //Set edges
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;
            if (!KFcounter.count(pKFi))
                continue;
            const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;
            g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);
            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);
            e->fx = pKFi->fx;
            e->fy = pKFi->fy;
            e->cx = pKFi->cx;
            e->cy = pKFi->cy;
            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);
            vpMapPointEdgeMono.push_back(pMP);
        }
    }

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore)
    {

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        // Optimize again without the outliers

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        MapPoint *pMP = vpMapPointEdgeStereo[i];

        if (e->chi2() > 7.815 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    if (!vToErase.empty())
    {
        for (size_t i = 0; i < vToErase.size(); i++)
        {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKF = *lit;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}

} //namespace ORB_SLAM

