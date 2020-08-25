# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# * 50Hz IMU; have trouble tracking the 90 deg turning
# SeqStartTime = [0] 
# SeqDuration = [800]
# SeqNameList = ['2019-02-05-15-01-16'];

# 50Hz IMU; crash couple of times; obselete
# SeqStartTime = [0] 
# SeqDuration = [800]
# SeqNameList = ['2019-02-05-15-13-52'];

# 50Hz IMU; best mapping so far
# SeqStartTime = [5] 
# SeqDuration = [800]
# SeqNameList = ['2019-02-05-18-58-08'];

# 200Hz IMU; best mapping so far
# SeqStartTime = [0] 
# SeqDuration = [800]
# SeqNameList = ['2019-02-06-18-29-27'];

# 200Hz IMU; have trouble tracking the 90 deg turning
# SeqStartTime = [0] 
# SeqDuration = [800]
# SeqNameList = ['2019-02-06-18-54-16'];

# * 200Hz IMU; extend from previous route
# SeqStartTime = [0] 
# SeqDuration = [823] 
# SeqNameList = ['2019-02-08-14-39-01'];

##
## All previous sequences are utilized to create the map
## Following are several testing sequences that have partial
## overlapping with the map
##
# 200Hz IMU; further extend from previous route; has issue
SeqStartTime = [0, 0, 0] 
SeqDuration = [530, 530, 1170]
SeqNameList = ['2019-02-08-17-16-08', '2019-05-03-17-48-01', '2019-05-07-19-46-48'];
# SeqStartTime = [0] 
# SeqDuration = [1170]
# SeqNameList = ['2019-05-07-19-46-48'];

# DEBUG
# SeqStartTime = [0] 
# SeqDuration = [180] 
# SeqNameList = ['2019-02-08-14-39-01-p1'];

# All valid sequences
# SeqStartTime = [0,5,0,0,0,0] 
# SeqDuration = [800,800,800,800,900,900]
# SeqNameList = ['2019-02-05-15-01-16', '2019-02-05-18-58-08', '2019-02-06-18-54-16', '2019-02-06-18-29-27', '2019-02-08-14-39-01', '2019-02-08-17-16-08'];

# Result_root = '/mnt/DATA/tmp/Turtlebot/Debug/'
# Result_root = '/mnt/DATA/tmp/Turtlebot/Lmk_800/ORB2_Stereo_GF_v2/'
# Result_root = '/mnt/DATA/tmp/Turtlebot/Lmk_800/ORB2_Stereo_GF_gpu/'
# Result_root = '/mnt/DATA/tmp/Turtlebot/Lmk_800/ORB2_Stereo_MapHash_v2/'
Result_root = '/mnt/DATA/tmp/Turtlebot/Lmk_800/ORB2_Stereo_Comb_v2/'
# Result_root = '/mnt/DATA/tmp/Turtlebot/Lmk_800/ORB2_Stereo_Comb_gpu/'
# Result_root = '/mnt/DATA/tmp/Turtlebot/Lmk_800/ORB2_Stereo_GF_gpu/'
# Result_root = '/mnt/DATA/tmp/Turtlebot/ORB2_Stereo_Baseline/'
# Result_root = '/mnt/DATA/tmp/Turtlebot/ORB2_Stereo_GT/'

gen_Map = False # True
doRect 	= str('false') # str('true') # 
doViz 	= str('true') # str('false') # 

# Number_GF_List = [1200] # [800]; # [400, 600, 800, 1000, 1500, 2000]; # 
Number_GF_List = [120] # [100, 130, 160, 200]; # 

Num_Repeating = 1 # 10 # 3 # 
SleepTime = 1 # 10 # 25

config_path = '/home/yipuzhao/ros_workspace/package_dir/ORB_Data'

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

            File_Setting = config_path + '/MYNT_yaml/mynteye_stereo_v2.yaml'

            File_Vocab   = config_path + '/ORBvoc.bin'
            File_rosbag  = '/mnt/DATA/Datasets/Turtlebot/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName
            # File_map = File_traj + '_map3D'
            File_map = '/mnt/DATA/tmp/Turtlebot/MYNT_4th_floor_map3D_run2'

            cmd_slam   = str('rosrun gf_orb_slam2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' \
                + str(int(num_gf*2)) + ' ' + doRect + ' ' + doViz + ' /mynteye/left/image_raw /mynteye/right/image_raw ' \
                + File_traj + ' ' + File_map)
            
            if gen_Map:
                # play with slow mo to improve mapping quality
                cmd_rosbag = 'rosbag play ' + File_rosbag + ' -s ' + str(SeqStartTime[sn]) + ' -u ' \
                + str(SeqDuration[sn]) + ' -r 0.2' + ' --pause'
            else:
                # wait for loading the map before running
                cmd_rosbag = 'rosbag play ' + File_rosbag + ' -s ' + str(SeqStartTime[sn]) + ' -u ' \
                + str(SeqDuration[sn]) # + ' -r 0.2' #  + ' --pause'

            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
            time.sleep(SleepTime)
            
            print bcolors.OKGREEN + "Loading the map from file system" + bcolors.ENDC
            time.sleep(90)

            if gen_Map:
                # logging the rosbag with published map
                print bcolors.OKGREEN + "Start logging rosbag with map" + bcolors.ENDC
                subprocess.Popen('roslaunch ../log_Map3D.launch path_data_logging:=' + File_map, shell=True)
                time.sleep(SleepTime)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)
            
            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill Stereo', shell=True)
            if gen_Map:
                print bcolors.OKGREEN + "Saving the map to file system" + bcolors.ENDC
                time.sleep(60)
                # kill map logging node
                print bcolors.OKGREEN + "Finish logging rosbag with map" + bcolors.ENDC
                subprocess.call('rosnode kill map_logging', shell=True)
            else:
                time.sleep(SleepTime)
            subprocess.call('pkill Stereo', shell=True)
