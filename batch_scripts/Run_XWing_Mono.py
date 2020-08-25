# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal


# SeqNameList = ['eo_wide_left-1526659828', 'eo_wide_right-1526659828', 'eo_wide_front-1526659828'];
# CamList = ['left', 'right', 'front'];
SeqNameList = ['eo_wide_left-1526659828-s1'];
CamList = ['left'];
# SeqNameList = ['eo_wide_front-1526659828_with_odom'];
# CamList = ['front'];
# SeqNameList = ['eo_wide_right-1526659828'];
# CamList = ['right'];

# Result_root = '/mnt/DATA/tmp/XWing/ORBv1_Baseline/'
Result_root = '/mnt/DATA/tmp/XWing/ORBv1_LC_Baseline/'

# Number_GF_List = [60, 80, 100, 130, 160, 200, 240]; # [80, 100, 120]; # 
Number_GF_List = [1500] # [400, 600, 800, 1000, 1500, 2000]; # 
Num_Repeating = 3 # 5 # 10 # 20 #  

SleepTime = 2 # 10 # 25

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

for sn, sname in enumerate(SeqNameList):

    for ri, num_gf in enumerate(Number_GF_List):

        for iteration in range(0, Num_Repeating):
        
            Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

            Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
            cmd_mkdir = 'mkdir ' + Experiment_dir
            subprocess.call(cmd_mkdir, shell=True)

            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn]
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_Setting = config_path + '/XWing_yaml/eo_' + CamList[sn] + '.yaml'

            File_Vocab = config_path + '/ORBvoc.bin'
            File_rosbag  = '/media/yipuzhao/651A6DA035A51611/' + SeqName + '.bag'
            
            File_traj = Experiment_dir + '/' + SeqName

            # enable viz
            cmd_slam   = str('rosrun gf_orb_slam2 Mono ' + File_Vocab + ' ' + File_Setting + ' '  + str(int(Number_GF_List[ri]*2)) + ' true /' + CamList[sn] + '_cam/image_raw ' + File_traj)
            # disable viz
            # cmd_slam   = str('rosrun gf_orb_slam2 Mono ' + File_Vocab + ' ' + File_Setting + ' '  + str(int(Number_GF_List[ri]*2)) + ' false /' + CamList[sn] + '_cam/image_raw ' + File_traj)
            cmd_rosbag = 'rosbag play ' + File_rosbag + ' -s 780' # + ' -r 0.2'
            
            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
            time.sleep(SleepTime)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill GF_ORB_SLAM', shell=True)
            # subprocess.call('pkill GF_ORB_SLAM', shell=True)
