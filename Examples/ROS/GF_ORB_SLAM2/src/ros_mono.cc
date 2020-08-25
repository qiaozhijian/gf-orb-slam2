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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include"../../../../include/System.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

//
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/publisher.h>


using namespace std;

//#define USE_FISHEYE_DISTORTION

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void GrabOdom(const nav_msgs::Odometry::ConstPtr& msg);

    void GrabPath(const nav_msgs::Path::ConstPtr    & msg);

    ORB_SLAM2::System* mpSLAM;
	
  double timeStamp;  
  cv::Mat Tmat;

    ros::Publisher mpCameraPosePublisher, mpCameraPoseInIMUPublisher;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 7)
    {
        cerr << endl << "Usage: rosrun gf_orb_slam2 Mono path_to_vocabulary path_to_settings budget_per_frame "
             << " do_viz topic_image_raw path_to_traj" << endl;
        ros::shutdown();
        return 1;
    }

    bool do_viz;
    stringstream s1(argv[4]);
    s1 >> boolalpha >> do_viz;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,do_viz);

    SLAM.SetConstrPerFrame(std::atoi(argv[3]));

#ifdef LOGGING_KF_LIST
    std::string fNameRealTimeBA = std::string(argv[6]) + "_Log_BA.txt";
    std::cout << std::endl << "Saving BA Log to Log_BA.txt" << std::endl;
#endif

#ifdef REALTIME_TRAJ_LOGGING
    std::string fNameRealTimeTrack = std::string(argv[6]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.SetRealTimeFileStream(fNameRealTimeTrack);
#endif

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("/cam0/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub = nh.subscribe(argv[5], 1, &ImageGrabber::GrabImage, &igb);

//
// ros::Subscriber sub2 = nh.subscribe("/odom", 100, &ImageGrabber::GrabOdom, &igb);
ros::Subscriber sub2 = nh.subscribe("/desired_path", 100, &ImageGrabber::GrabPath, &igb);

// TODO
    // figure out the proper queue size
igb.mpCameraPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose", 100);
igb.mpCameraPoseInIMUPublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose_in_imu", 100);
  

while(ros::ok()) 
    ros::spin();
    // ros::spin();

    cout << "ros_mono: done with spin!" << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM( std::string(argv[6]) + "_KeyFrameTrajectory.txt" );
    SLAM.SaveTrackingLog( std::string(argv[6]) + "_Log.txt" );
#ifdef LOCAL_BA_TIME_LOGGING
    SLAM.SaveMappingLog( std::string(argv[6]) + "_Log_Mapping.txt" );
#endif


    std::cout << "Finished saving!" << std::endl;

    ros::shutdown();

    cout << "ros_mono: done with ros Shutdown!" << endl;

    // Stop all threads
    SLAM.Shutdown();

    cout << "ros_mono: done with SLAM Shutdown!" << endl;

    return 0;
}


void ImageGrabber::GrabOdom(const nav_msgs::Odometry::ConstPtr& msg) {
  /*
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  */
  
  // TODO
  timeStamp = msg->header.stamp.toSec();
  
  mpSLAM->mpTracker->BufferingOdom(
    timeStamp, 
    msg->pose.pose.position.x, 
    msg->pose.pose.position.y, 
    msg->pose.pose.position.z, 
    msg->pose.pose.orientation.w, 
			    msg->pose.pose.orientation.x, 
			    msg->pose.pose.orientation.y, 
			    msg->pose.pose.orientation.z
  );
}

void ImageGrabber::GrabPath(const nav_msgs::Path::ConstPtr& msg) {
    /*
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    */

    //    // densify the path
    //    // create a cubic spline interpolator
    //    nav_msgs::Path path_dense;
    //    // pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
    //    path_smoothing::CubicSplineInterpolator csi(double(100.0),
    //                                                (unsigned int)0,
    //                                                true,
    //                                                true);
    //    csi.interpolatePath(*msg, path_dense);

    size_t N = msg->poses.size();
    //    ROS_INFO("Size of path: before [%d] vs. after [%d]", msg->poses.size(), N);
    for (size_t i=0; i<N; ++i) {

        timeStamp = msg->poses[i].header.stamp.toSec();
        mpSLAM->mpTracker->BufferingOdom(
                    timeStamp,
                    msg->poses[i].pose.position.x,
                    msg->poses[i].pose.position.y,
                    msg->poses[i].pose.position.z,
                    msg->poses[i].pose.orientation.w,
                    msg->poses[i].pose.orientation.x,
                    msg->poses[i].pose.orientation.y,
                    msg->poses[i].pose.orientation.z
                    );
    }

    //    mpDensePathPub.publish(path_dense);
}


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    if (pose.empty())
        return;

    // std::cout << "broadcast pose!" << std::endl;
/*
    /// broadcast tf
    // global left handed coordinate system 
    static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                                     -1, 1,-1, 1,
                                     -1,-1, 1, 1,
                                     1, 1, 1, 1);

    //prev_pose * T = pose
    cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();

    tf::Matrix3x3 tf3d;
    tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
                  pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
                  pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

    tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                            0, 0, 1,
                                            1, 0, 0);

    static tf::TransformBroadcaster br;

    tf::Matrix3x3 globalRotation_rh = tf3d;
    tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;

    tf::Quaternion tfqt;
    globalRotation_rh.getRotation(tfqt);

    double aux = tfqt[0];
    tfqt[0]=-tfqt[2];
    tfqt[2]=tfqt[1];
    tfqt[1]=aux;

    tf::Transform transform;
    transform.setOrigin(globalTranslation_rh);
    transform.setRotation(tfqt);

    br.sendTransform(tf::StampedTransform(transform, cv_ptr->header.stamp, "map", "camera_pose"));
    */
    
    
    /// broadcast campose pose message
    // camera pose
 /*
         tf::Matrix3x3 R( 0,  0,  1,
                        -1,  0,  0,
                         0, -1,  0);
         tf::Transform T ( R * tf::Matrix3x3( transform.getRotation() ), R * transform.getOrigin() );
  */
 
     cv::Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*pose.rowRange(0,3).col(3);
        tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                        Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                        Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
        tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

        tf::Transform tfTcw(M,V);
    geometry_msgs::Transform gmTwc;
        tf::transformTFToMsg(tfTcw, gmTwc);
        
        geometry_msgs::Pose camera_pose;
        camera_pose.position.x = gmTwc.translation.x;
        camera_pose.position.y = gmTwc.translation.y;
        camera_pose.position.z = gmTwc.translation.z;
        camera_pose.orientation = gmTwc.rotation;
    
    geometry_msgs::PoseWithCovarianceStamped camera_odom;
        camera_odom.header.frame_id = "odom";
        camera_odom.header.stamp = cv_ptr->header.stamp;
        camera_odom.pose.pose = camera_pose;
    
        mpCameraPosePublisher.publish(camera_odom);
    
    tf::Matrix3x3 Ric( 0,  0,  1,
                         -1,  0,  0,
                         0,  -1,  0);
    
    tf::Matrix3x3 Rbi( 0,  -1,  0,
                         0,  0,  -1,
                         1,  0,  0);
     
        tf::Transform tfTiw ( Ric * tf::Matrix3x3( tfTcw.getRotation() ) * Rbi, Ric * tfTcw.getOrigin() );
    geometry_msgs::Transform gmTwi;
        tf::transformTFToMsg(tfTiw, gmTwi);
        
        geometry_msgs::Pose camera_pose_in_imu;
        camera_pose_in_imu.position.x = gmTwi.translation.x;
        camera_pose_in_imu.position.y = gmTwi.translation.y;
        camera_pose_in_imu.position.z = gmTwi.translation.z;
        camera_pose_in_imu.orientation = gmTwi.rotation;
    
    geometry_msgs::PoseWithCovarianceStamped camera_odom_in_imu;
        camera_odom_in_imu.header.frame_id = "odom";
        camera_odom_in_imu.header.stamp = cv_ptr->header.stamp;
        camera_odom_in_imu.pose.pose = camera_pose_in_imu;
    
        mpCameraPoseInIMUPublisher.publish(camera_odom_in_imu);

}
