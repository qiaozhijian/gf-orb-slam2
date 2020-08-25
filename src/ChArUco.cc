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

namespace ORB_SLAM2
{

ChArUco::ChArUco(const cv::Mat &camMatrix_, 
                 const cv::Mat &distCoeffs_,
                 const std::string &paramsFile_,
                 const bool &showRejected_,
                 const bool &refindStrategy_)
{
    
    showRejected = showRejected_;
    refindStrategy = refindStrategy_;

    camMatrix_.copyTo(camMatrix);
    distCoeffs_.copyTo(distCoeffs);

    if (loadParameters(paramsFile_)) {
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

        charucoboard = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
        board = charucoboard.staticCast<cv::aruco::Board>();
        std::cout << "succeed to load chArUco parameters!" << std::endl;
    }
    else {
        // failed
        std::cout << "failed to load chArUco parameters!" << std::endl;
    }
}

bool ChArUco::loadParameters(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    
    detectorParams = cv::aruco::DetectorParameters::create();
    //
    fs["adaptiveThreshWinSizeMin"] >> detectorParams->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> detectorParams->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> detectorParams->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> detectorParams->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> detectorParams->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> detectorParams->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> detectorParams->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> detectorParams->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> detectorParams->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> detectorParams->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> detectorParams->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> detectorParams->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> detectorParams->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> detectorParams->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> detectorParams->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> detectorParams->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> detectorParams->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> detectorParams->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> detectorParams->minOtsuStdDev;
    fs["errorCorrectionRate"] >> detectorParams->errorCorrectionRate;
    //
    fs["squaresX"] >> squaresX;
    fs["squaresY"] >> squaresY;
    fs["dictionaryId"] >> dictionaryId;
    fs["squareLength"] >> squareLength;
    fs["markerLength"] >> markerLength;
    
    return true;
}

bool ChArUco::process(const cv::Mat& image, cv::Mat & Twc) {

    std::vector< int > markerIds, charucoIds;
    // This is the most important part about ORB SLAM the vector vector markerCorners
    std::vector< std::vector< cv::Point2f > > markerCorners, rejectedMarkers;
    std::vector< cv::Point2f > charucoCorners;
    
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams, rejectedMarkers);
    if (refindStrategy)
        cv::aruco::refineDetectedMarkers(image, board, markerCorners, markerIds, rejectedMarkers, camMatrix, distCoeffs);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if (markerIds.size() > 0){
        interpolatedCorners = cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoboard, charucoCorners, charucoIds, camMatrix, distCoeffs);
    }

    // estimate charuco board pose
    bool validPose = false;
    cv::Vec3d rvec, tvec;
    if (camMatrix.total() != 0){
        validPose = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard, camMatrix, distCoeffs, rvec, tvec);

        // Print out
        std::cout << "rvec = " << rvec <<std::endl;
        std::cout << "tvec = " << tvec <<std::endl;

        // Assemble into homo matrix
        cv::Mat Rmat = cv::Mat(3, 3, CV_32F);
        cv::Rodrigues(rvec, Rmat);
        Rmat.copyTo(Twc.rowRange(0,3).colRange(0,3));
        Twc.at <float>(0,3) = tvec [0];
        Twc.at <float>(1,3) = tvec [1];
        Twc.at <float>(2,3) = tvec [2];
    }

#ifdef VIZ_ARUCO
    // draw results
    image.copyTo(image_viz);
    if (markerIds.size() > 0) {
        cv::aruco::drawDetectedMarkers(image_viz, markerCorners);
    }

    if (showRejected && rejectedMarkers.size() > 0)
        cv::aruco::drawDetectedMarkers(image_viz, rejectedMarkers, cv::noArray(), cv::Scalar(100, 0, 255));

    if (interpolatedCorners > 0) {
        cv::Scalar color;
        color = cv::Scalar(255, 0, 0);
        cv::aruco::drawDetectedCornersCharuco(image_viz, charucoCorners, charucoIds, color);
    }

    if (validPose) {
        float axisLength = 0.5f * ((float)std::min(squaresX, squaresY) * (squareLength));
        cv::aruco::drawAxis(image_viz, camMatrix, distCoeffs, rvec, tvec, axisLength);
    }
    
    cv::imwrite( "/home/yipu/catkin_ws/src/GF_ORB_SLAM2/charuco_Image.jpg", image_viz );
    cv::namedWindow( "Gray image", cv::WINDOW_AUTOSIZE );
    cv::imshow("chArUco_board_detection", image_viz);
    cvWaitKey(0);

#endif
    
    return validPose;
}

}

