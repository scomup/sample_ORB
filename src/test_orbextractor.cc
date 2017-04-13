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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include <boost/format.hpp> 

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;



int main(int argc, char **argv)
{

    cv::Mat img;
    double timestamp = 0.0;

    ORB_SLAM2::System SLAM(
                           "/home/liu/workspace/sample_ORB/Vocabulary/ORBvoc.txt",
                           "/home/liu/workspace/sample_ORB/Examples/Monocular/Settings.yaml",
                           ORB_SLAM2::System::MONOCULAR,true);

    for(int ni=0; ni<250; ni++)
    {
      usleep(333333);
      std::string str = (boost::format("/home/liu/workspace/Datasets/example/frame%04d.jpg") % ni).str();
      img = cv::imread(str);
        if(img.empty())
        {
            cerr << endl << "Failed to load image at: " << str << endl;
            return 1;
        }
       
        SLAM.TrackMonocular(img,timestamp);
        timestamp += 0.033;
    }

    // Stop all threads
    SLAM.Shutdown();


    return 0;
}
