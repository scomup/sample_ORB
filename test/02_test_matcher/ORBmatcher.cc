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

#include "ORBmatcher.h"

#include <limits.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <stdint-gcc.h>

using namespace std;

namespace sample_ORB
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri) : mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}



int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{
    int nmatches = 0;
    vnMatches12 = vector<int>(F1.mvKeysUn.size(), -1);

    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeysUn.size(), INT_MAX);
    vector<int> vnMatches21(F2.mvKeysUn.size(), -1);

    for (size_t i1 = 0, iend1 = F1.mvKeysUn.size(); i1 < iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;

        //Only search the matching for level0
        if (level1 > 0)
            continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize, level1, level1);

        if (vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for (vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1, d2);

            if (vMatchedDistance[i2] <= dist)
                continue;

            if (dist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = dist;
                bestIdx2 = i2;
            }
            else if (dist < bestDist2)
            {
                bestDist2 = dist;
            }
        }

        if (bestDist <= TH_LOW)
        {
            if (bestDist < (float)bestDist2 * mfNNratio)
            {
                //Make sure this keyPoint not matched with other point.
                if (vnMatches21[bestIdx2] >= 0)
                {
                    vnMatches12[vnMatches21[bestIdx2]] = -1;
                    nmatches--;
                }
                vnMatches12[i1] = bestIdx2;
                vnMatches21[bestIdx2] = i1;
                vMatchedDistance[bestIdx2] = bestDist;
                nmatches++;

                if (mbCheckOrientation)
                {
                    float rot = F1.mvKeysUn[i1].angle - F2.mvKeysUn[bestIdx2].angle;
                    if (rot < 0.0)
                        rot += 360.0f;
                    int bin = round(rot * factor);
                    if (bin == HISTO_LENGTH)
                        bin = 0;
                    assert(bin >= 0 && bin < HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }
    }

    if (mbCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for (int i = 0; i < HISTO_LENGTH; i++)
        {
            if (i == ind1 || i == ind2 || i == ind3)
                continue;
            for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
            {
                int idx1 = rotHist[i][j];
                if (vnMatches12[idx1] >= 0)
                {
                    vnMatches12[idx1] = -1;
                    nmatches--;
                }
            }
        }
    }

    //Update the location of matched points, in order to seek matched points for subsequent frame easily.
    for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
        if (vnMatches12[i1] >= 0)
            vbPrevMatched[i1] = F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}

/*
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0, 3).col(3);

    const cv::Mat twc = -Rcw.t() * tcw;

    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0, 3).col(3);

    const cv::Mat tlc = Rlw * twc + tlw;

    const bool bForward = tlc.at<float>(2) > CurrentFrame.mb && !bMono;
    const bool bBackward = -tlc.at<float>(2) > CurrentFrame.mb && !bMono;

    for (int i = 0; i < LastFrame.N; i++)
    {
        MapPoint *pMP = LastFrame.mvpMapPoints[i];

        if (pMP)
        {
            if (!LastFrame.mvbOutlier[i])
            {
                // Project
                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0 / x3Dc.at<float>(2);

                if (invzc < 0)
                    continue;

                float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
                float v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;

                if (u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX)
                    continue;
                if (v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY)
                    continue;

                int nLastOctave = LastFrame.mvKeys[i].octave;

                // Search in a window. Size depends on scale
                float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

                vector<size_t> vIndices2;

                if (bForward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave);
                else if (bBackward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, 0, nLastOctave);
                else
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave - 1, nLastOctave + 1);

                if (vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for (vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end(); vit != vend; vit++)
                {
                    const size_t i2 = *vit;
                    if (CurrentFrame.mvpMapPoints[i2])
                        if (CurrentFrame.mvpMapPoints[i2]->Observations() > 0)
                            continue;

                    if (CurrentFrame.mvuRight[i2] > 0)
                    {
                        const float ur = u - CurrentFrame.mbf * invzc;
                        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                        if (er > radius)
                            continue;
                    }

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP, d);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdx2 = i2;
                    }
                }

                if (bestDist <= TH_HIGH)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
                    nmatches++;

                    if (mbCheckOrientation)
                    {
                        float rot = LastFrame.mvKeysUn[i].angle - CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }

    //Apply rotation consistency
    if (mbCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for (int i = 0; i < HISTO_LENGTH; i++)
        {
            if (i != ind1 && i != ind2 && i != ind3)
            {
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}*/

void ORBmatcher::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1 = 0;
    int max2 = 0;
    int max3 = 0;

    for (int i = 0; i < L; i++)
    {
        const int s = histo[i].size();
        if (s > max1)
        {
            max3 = max2;
            max2 = max1;
            max1 = s;
            ind3 = ind2;
            ind2 = ind1;
            ind1 = i;
        }
        else if (s > max2)
        {
            max3 = max2;
            max2 = s;
            ind3 = ind2;
            ind2 = i;
        }
        else if (s > max3)
        {
            max3 = s;
            ind3 = i;
        }
    }

    if (max2 < 0.1f * (float)max1)
    {
        ind2 = -1;
        ind3 = -1;
    }
    else if (max3 < 0.1f * (float)max1)
    {
        ind3 = -1;
    }
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist = 0;

    for (int i = 0; i < 8; i++, pa++, pb++)
    {
        unsigned int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

} //namespace ORB_SLAM
