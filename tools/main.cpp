/**
* This file is part of GF-ORB-SLAM2.
*
* Copyright (C) 2019 Yipu Zhao <yipu dot zhao at gatech dot edu> 
* (Georgia Institute of Technology)
* For more information see 
* <https://sites.google.com/site/zhaoyipu/good-feature-visual-slam>
*
* GF-ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GF-ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GF-ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ChArUco.h"

int main(int argc, char **argv){

    ChArUco *obj = new ChArUco();
    obj->readCameraParameters("/home/parallels/opencv_contrib/modules/aruco/samples/calibrate_camera.yml", obj->camMatrix, obj->distCoeffs);
    obj->readDetectorParameters("/home/parallels/opencv_contrib/modules/aruco/samples/detector_params.yml", obj->detectorParams);

    ros::init(argc,argv,"Display_Images");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/front/image_raw",1000,&ChArUco::process, obj);

    ros::spin();
    return 0;
}
