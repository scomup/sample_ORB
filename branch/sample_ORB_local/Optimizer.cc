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

#include <mutex>

namespace sample_ORB
{

class PoseSolver
{
  public:
    PoseSolver(Frame *pFrame);
    int Solve();

  protected:
    void Update();
    Eigen::Matrix<double, 2, 3> Jacobiani(cv::Mat point);
    Eigen::Matrix<double, 2, 1> Errori(cv::Mat point, double u, double v);
    double TotalError();
    void ReadDataFromFrame();
    double mFx;
    double mFy;
    double mCx;
    double mCy;
    double mYaww;
    double mXw;
    double mYw;
    double mDyaw;
    double mDx;
    double mDy;
    Frame *mpFrame;
};

PoseSolver::PoseSolver(Frame *pFrame) : mpFrame(pFrame)
{
    ReadDataFromFrame();
}

Eigen::Matrix<double, 2, 3> PoseSolver::Jacobiani(cv::Mat point)
{
    double x = point.at<float>(0);
    double y = point.at<float>(1);
    double z = point.at<float>(2);
    Eigen::Matrix<double, 2, 3> J;
    double sinY = sin(mYaww - mDyaw);
    double cosY = cos(mYaww - mDyaw);
    J << -(mFx / z),
        0,
        -(mFx / z) * sinY * (x - mXw) + (mFx / z) * cosY * (y - mYw),
        0,
        -(mFy / z),
        -(mFy / z) * cosY * (x - mXw) - (mFy / z) * sinY * (y - mYw);
    return J;
}

Eigen::Matrix<double, 2, 1> PoseSolver::Errori(cv::Mat point, double u, double v)
{
    Eigen::Matrix<double, 2, 1> ei;
    double x = point.at<float>(0);
    double y = point.at<float>(1);
    double z = point.at<float>(2);
    double sinY = sin(mYaww - mDyaw);
    double cosY = cos(mYaww - mDyaw);

    ei << u - (mFx / z) * (cosY * (x - mXw) + sinY * (y - mYw) + mDx) - mCx,
        v - (mFy / z) * (-sinY * (x - mXw) + cosY * (y - mYw) + mDy) - mCy;
    return ei;
}

double PoseSolver::TotalError()
{
    double tot_error = 0;
    const int N = mpFrame->N;
    for (int i = 0; i < N; i++)
    {
        MapPoint *pMP = mpFrame->mvpMapPoints[i];
        if (pMP && mpFrame->mvbOutlier[i] == false)
        {
            const cv::KeyPoint &kpUn = mpFrame->mvKeysUn[i];
            Eigen::Matrix<double, 2, 1> Ei = Errori(pMP->GetWorldPos(), kpUn.pt.x, kpUn.pt.y);
            Eigen::Matrix<double, 1, 1> E2 = Ei.transpose() * Ei;
            tot_error = tot_error + E2(0, 0);
        }
    }
    return tot_error;
}

void PoseSolver::Update()
{
    cv::Mat Rcw = Converter::computeMatrixFromAngles(0, 0, mDyaw - mYaww);
    cv::Mat Tcw = mpFrame->mTcw.clone();
    Tcw.at<float>(0,3) += mDx;
    Tcw.at<float>(1,3) += mDy;
    Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
    mpFrame->SetPose(Tcw);

}

int PoseSolver::Solve()
{
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
                Eigen::Matrix<double, 2, 3> Ji = Jacobiani(pMP->GetWorldPos());
                Eigen::Matrix<double, 2, 1> Ei = Errori(pMP->GetWorldPos(), kpUn.pt.x, kpUn.pt.y);
                H = H + Ji.transpose() * Ji/10000;
                B = B + Ji.transpose() * Ei/10000;
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
        double tot_error = TotalError();
        //printf("bef:%f\n", tot_error);
        mDx += arg(0, 0);
        mDy += arg(1, 0);
        mDyaw += arg(2, 0);
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
        tot_error = TotalError();
        //printf("aft:%f\n", tot_error);
        if (last_error - tot_error < 0.1)
        {
            last_error = tot_error;
            std::cout << "The error is converge:" << std::endl;
            break;
        }
        else
        {
            last_error = tot_error;
        }
    }
     Update();
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
    cv::Mat twc = mpFrame->GetCameraCenter();
    cv::Mat Rwc = mpFrame->GetRotationInverse();
    float pitch, roll, yaw;
    Converter::computeAnglesFromMatrix(Rwc, pitch, roll, yaw);
    mXw = twc.at<float>(0);
    mYw = twc.at<float>(1);
    mYaww = (float)yaw;
    mDyaw = 0;
    mDx = 0;
    mDy = 0;
}
}

namespace sample_ORB
{
using namespace std;
#if 0

int Optimizer::PoseOptimization(Frame *pFrame)
{
    PoseSolver solver(pFrame);
    int n = solver.Solve();
    cv::Mat twc = pFrame->GetCameraCenter();
    std::cout<<twc<<std::endl;
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
    std::cout<<twc<<std::endl;
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
