# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# Corridor, failed to close loop between 2 runs
# SeqStartTime = [210] # [300] # 
# SeqDuration = [300]
# SeqNameList = ['2019-01-25-12-27_stereo'];

# * Corridor (short; no loop)
# SeqNameList = ['2019-01-23-06-22_stereo'];
# SeqStartTime = [0] 
# SeqDuration = [100]

# * Corridor (short)
# SeqStartTime = [250]
# SeqDuration = [100]
# SeqNameList = ['2019-01-24-15-09_stereo'];

# * Corridor (2 loops)
# SeqStartTime = [0] 
# SeqDuration = [300]
# SeqNameList = ['2019-01-25-15-10_stereo'];

# * Room I
# SeqStartTime = [60] 
# SeqDuration = [130]
# SeqNameList = ['2019-01-25-17-30_stereo'];

# * Room II
# SeqStartTime = [230] 
# SeqDuration = [999]
# SeqNameList = ['2019-01-24-18-09_stereo'];

# * TSRB
SeqStartTime = [238] 
SeqDuration = [435]
SeqNameList = ['2019-03-01-21-42_stereo'];

# * Batch Eval
# SeqStartTime = [0, 60, 230] 
# SeqDuration = [300, 130, 999]
# SeqNameList = ['2019-01-25-15-10_stereo', '2019-01-25-17-30_stereo', '2019-01-24-18-09_stereo'];

Result_root = '/mnt/DATA/tmp/Hololens/ORB2_Stereo_Baseline/'
# Result_root = '/mnt/DATA/tmp/Hololens/ORB2_Stereo_GT/'

Number_GF_List = [1500]; # [400, 600, 800, 1000, 1500, 2000]; # 
# Number_GF_List = [60, 80, 100, 150, 200];
Num_Repeating = 10 # 5 # 1
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
        # mkdir for map as well
        cmd_mkdir = 'mkdir -p ' + Experiment_dir + '_Map'
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn] # + '_blur_5'
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_Setting = config_path + '/Hololens_yaml/Hololens_stereo_lmk800.yaml'
            # File_Setting = config_path + '/Hololens_yaml/Hololens_stereo_lmk800_old.yaml'

            File_Vocab   = config_path + '/ORBvoc.bin'
            # File_rosbag  = '/media/yipuzhao/651A6DA035A51611/Hololens/BagFiles/' + SeqName + '.bag'
            File_rosbag  = '/mnt/DATA/Datasets/Hololens/BagFiles/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName
            File_map = File_traj + '_Map'

            # pre-rect
            # cmd_slam   = str('rosrun gf_orb_slam2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf*2)) + ' true true /left_cam/image_raw /right_cam/image_raw ' + File_traj + ' ' + File_map))
            # do viz
            cmd_slam   = str('rosrun gf_orb_slam2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf*2)) + ' false true /left_cam/image_raw /right_cam/image_raw ' + File_traj + ' ' + File_map)
            # no viz
            # cmd_slam   = str('rosrun gf_orb_slam2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf*2)) + ' false false /left_cam/image_raw /right_cam/image_raw ' + File_traj + ' ' + File_map))
            cmd_rosbag = 'rosbag play ' + File_rosbag + ' -s ' + str(SeqStartTime[sn]) + ' -u ' + str(SeqDuration[sn]) + ' -r 0.3'

            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
            time.sleep(SleepTime)
            
            # print bcolors.OKGREEN + "Loading the map from file system" + bcolors.ENDC
            # time.sleep(20)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill Stereo', shell=True)
            time.sleep(SleepTime)
            # print bcolors.OKGREEN + "Saving the map to file system" + bcolors.ENDC
            # time.sleep(20)
            subprocess.call('pkill Stereo', shell=True)

