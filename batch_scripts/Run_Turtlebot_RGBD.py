# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# * Corridor (3 loops)
# SeqStartTime = [30]  # [480] # 
# SeqDuration = [860]
# SeqNameList = ['2019-02-01-19-09-24'];
SeqStartTime = [0]  # [480] # 
SeqDuration = [999]
SeqNameList = ['2019-04-04-17-17-25_RGBD'];

# Result_root = '/mnt/DATA/tmp/Turtlebot/ORB2_RGBD_Debug/'
# Result_root = '/mnt/DATA/tmp/Turtlebot/ORB_RGBD/'
Result_root = '/mnt/DATA/tmp/Turtlebot/Lmk800/GF_RGBD/'
# Result_root = '/mnt/DATA/tmp/Turtlebot/Lmk800/GF_RGBD_gpu/'

gen_Map = False # True

# Number_GF_List = [800]; # [400, 600, 800, 1000, 1500, 2000]; # 
Number_GF_List = [100, 130, 160, 200] # [60, 80, 100, 150, 200];

Num_Repeating = 3 # 1 # 10 # 5 # 
SleepTime = 1 # 10 # 25

config_path = '/home/yipuzhao/catkin_ws/src/ORB_Data'
# config_path = '/home/yipuzhao/ros_workspace/package_dir/ORB_Data'

#----------------------------------------------------------------------------------------------------------------------
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ALERT = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

for ri, num_gf in enumerate(Number_GF_List):
    
    Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        # mkdir for pose traj
        cmd_mkdir = 'mkdir -p ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn] # + '_blur_5'
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_Setting = config_path + '/kinect_turtlebot_rgbd.yaml'

            File_Vocab   = config_path + '/ORBvoc.bin'
            File_rosbag  = '/mnt/DATA/Datasets/Turtlebot/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName
            # File_map = File_traj + '_Map/'
            # File_map = '/mnt/DATA/tmp/Turtlebot/2019-02-01-19-09-24_Map_v4'
            File_map = '/home/yipuzhao/Kinect_Map/2019-02-01-19-09-24_Map_v4/'

            # do viz
            # cmd_slam   = str('rosrun gf_orb_slam2 RGBD ' + File_Vocab + ' ' + File_Setting + ' ' \
            #     + str(int(num_gf*2)) + ' true /camera/rgb/image_mono /camera/depth_registered/image_raw ' \
            #     + File_traj + ' ' + File_map)
            # no viz
            cmd_slam   = str('rosrun gf_orb_slam2 RGBD ' + File_Vocab + ' ' + File_Setting + ' ' \
                + str(int(num_gf*2)) + ' false /camera/rgb/image_mono /camera/depth_registered/image_raw ' \
                + File_traj + ' ' + File_map)
            
            if gen_Map:
                # play with slow mo to improve mapping quality
                cmd_rosbag = 'rosbag play --clock ' + File_rosbag + \
                ' -s ' + str(SeqStartTime[sn]) + ' -u ' + str(SeqDuration[sn]) + ' -r 0.2'
            else:
                # wait for loading the map before running
                cmd_rosbag = 'rosbag play --clock ' + File_rosbag + \
                ' -s ' + str(SeqStartTime[sn]) + ' -u ' + str(SeqDuration[sn]) # + ' --pause'

            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
            time.sleep(SleepTime)
            
            print bcolors.OKGREEN + "Loading the map from file system" + bcolors.ENDC
            time.sleep(60)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            if gen_Map:
                # logging the rosbag with published map
                print bcolors.OKGREEN + "Start logging rosbag with map" + bcolors.ENDC
                subprocess.Popen('roslaunch ../log_Map3D.launch path_data_logging:=' + File_map + 'map3D.bag', shell=True)
                time.sleep(5)
                print bcolors.OKGREEN + "Finish logging rosbag with map" + bcolors.ENDC
                subprocess.call('rosnode kill map_logging', shell=True)

            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill RGBD', shell=True)
            if gen_Map:
                print bcolors.OKGREEN + "Saving the map to file system" + bcolors.ENDC
                time.sleep(40)
            else:
                time.sleep(SleepTime)
            subprocess.call('pkill RGBD', shell=True)

