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

//#define DEBUG

#ifdef DEBUG
#define debug_printf(fmt, ...) printf(fmt, __VA_ARGS__)
#else
#define debug_printf(...)
#endif

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <list>
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

//#include "../../Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace sample_ORB
{


class Optimizer
{
public:
    int static PoseOptimization(Frame* pFrame);
    void static LocalBundleAdjustment(KeyFrame *pKF, std::list<KeyFrame*> lLocalKeyFrames, bool* pbStopFlag);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
