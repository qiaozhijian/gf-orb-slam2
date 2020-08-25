<!-- rosrun gf_orb_slam2 RGBD ../ORB_Data/ORBvoc.bin ../ORB_Data/kinect_turtlebot_rgbd.yaml 300 true /camera/rgb/image_mono /camera/depth_registered/image_raw /home/yipu/Debug_Log/test -->

rosrun gf_orb_slam2 RGBD ../ORB_Data/ORBvoc.bin ../ORB_Data/kinect_turtlebot_rgbd.yaml 1600 true /camera/rgb/image_mono /camera/depth_registered/image_raw /home/yipu/Debug_Log/test /mnt/DATA/tmp/Turtlebot/2019-02-01-19-09-24_Map_4

<!-- rosrun gf_orb_slam2 Stereo ../ORB_Data/ORBvoc.bin ../ORB_Data/mynteye_stereo.yaml 1600 false true /mynteye/left/image_raw /mynteye/right/image_raw /home/yipu/Debug_Log/test /home/yipu/Debug_Log/test_Map/ -->

rosrun gf_orb_slam2 Stereo ../ORB_Data/ORBvoc.bin ../ORB_Data/MYNT_yaml/mynteye_stereo_v2.yaml 240 false true /mynteye/left/image_raw /mynteye/right/image_raw /home/yipu/Debug_Log/test /home/yipu/MYNT_4th_floor_map3D_run2/