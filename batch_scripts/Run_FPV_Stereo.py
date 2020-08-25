# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

SeqNameList = ['indoor_forward_3', 'indoor_forward_5', 'indoor_forward_6', \
               'indoor_forward_7', 'indoor_forward_9', 'indoor_forward_10'];
# SeqNameList = ['indoor_forward_5'];

# Note
# when testing with pre-compute FAST, keep in mind that the detection results 
# of baseline ORB and GF are different: ORB detection is from rectified image,
# while GF detection is from distorted raw image.

# Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_GGraphAnticNH_Stereo_Speedx'
Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_GGraphAnticNLv2_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_GGraphAntic_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_GGraph_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_SWF_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_Covis_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_BaseBA_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/ORB_BaseBA_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/Debug_Stereo_Speedx'

Playback_Rate_List = [1.0] # [1.0, 2.0, 3.0] # [0.5, 1.0, 1.5] # [1.0, 2.0, 3.0, 4.0, 5.0];

# Optimal param for ORB
# Number_GF_List = [800]; 
# Number_GF_List = [600, 800, 1000, 1500]; # [200, 300, 400, 600, 800, 1000, 1500, 2000]; # [2000]; # 

# Optimal param for GF
Number_GF_List = [80]; # [130]; 
# Number_GF_List = [80, 100, 130, 160]; # [80, 100, 130, 160, 200, 240, 280, 320, 360]; #  

Num_Repeating = 10 # 1 # 

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

for pi, rate in enumerate(Playback_Rate_List):
    for ri, num_gf in enumerate(Number_GF_List):
        
        Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

        for iteration in range(0, Num_Repeating):

            Experiment_dir = Result_root + str(rate) + '/' \
             + Experiment_prefix + '_Round' + str(iteration + 1)
            cmd_mkdir = 'mkdir -p ' + Experiment_dir
            subprocess.call(cmd_mkdir, shell=True)

            for sn, sname in enumerate(SeqNameList):
                
                subprocess.call('clear all', shell=True)
                print bcolors.ALERT + "====================================================================" + bcolors.ENDC

                SeqName = SeqNameList[sn]
                print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

                File_Setting = '../../ORB_Data/UZH_FPV_yaml/UZH_FPV_stereo_lmk800.yaml'

                # File_Vocab   = '../ORB_Data/ORBvoc.txt'
                File_Vocab   = config_path + '/ORB_Data/ORBvoc.bin'
                File_rosbag  = '/mnt/DATA/Datasets/UZH_FPV/BagFiles/' + SeqName + '_snapdragon_with_gt.bag'
                File_traj = Experiment_dir + '/' + SeqName

                # do viz
                cmd_slam   = str('rosrun gf_orb_slam2 Stereo ' + File_Vocab + ' ' + File_Setting + ' '\
                    + str(int(num_gf)) + ' false true /cam0/image_raw /cam1/image_raw ' + File_traj)
                # no viz
                # cmd_slam   = str('rosrun gf_orb_slam2 Stereo ' + File_Vocab + ' ' + File_Setting + ' '\
                #     + str(int(num_gf)) + ' false false /cam0/image_raw /cam1/image_raw ' + File_traj)
                cmd_rosbag = 'rosbag play ' + File_rosbag + ' -r ' + str(rate) # + ' -u 20' 

                print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
                print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC

                print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
                proc_slam = subprocess.Popen(cmd_slam, shell=True)

                print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
                time.sleep(SleepTime)

                print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
                proc_bag = subprocess.call(cmd_rosbag, shell=True)

                print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
                subprocess.call('rosnode kill Stereo', shell=True)
                time.sleep(SleepTime)
                # print bcolors.OKGREEN + "Saving the map to file system" + bcolors.ENDC
                # time.sleep(15)
                subprocess.call('pkill Stereo', shell=True)

