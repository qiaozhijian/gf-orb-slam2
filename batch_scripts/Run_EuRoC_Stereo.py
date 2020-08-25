# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# SeqNameList = ['MH_01_easy', 'MH_03_medium', 'MH_05_difficult', 'V2_02_medium', 'not_exist'];
# SeqNameList = ['V1_03_difficult', 'V2_02_medium', 'V2_03_difficult', 'not_exist'];
# SeqNameList = ['MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult', 'MH_05_difficult', 'V1_01_easy', 'V1_02_medium', 'V1_03_difficult', 'V2_01_easy', 'V2_02_medium', 'V2_03_difficult', 'not_exist'];
SeqNameList = ['V1_01_easy', 'not_exist'];

Result_root = '/media/qzj/Document/grow/research/slamDataSet/EuRoC/GF_ORB2_Stereo_Baseline/'
# Result_root = '/mnt/DATA/tmp/EuRoC/Lmk_800/ORB2_active_measErr_withFrameInfo/'

Number_GF_List = [800]; # [400, 600, 800, 1000, 1500, 2000]; # 
# Number_GF_List = [60, 80, 100, 150, 200];
Num_Repeating = 1 # 10 # 20 #  5 # 
SleepTime = 10 # 10 # 25

config_path = '/media/qzj/Software/code/catkin_ws/src/ORB_Data'

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

# subprocess.Popen('roscore', shell=True)
# time.sleep(SleepTime)


subprocess.Popen('rosrun rviz rviz -d /media/qzj/Software/code/catkin_ws/src/gf_orb_slam2/rviz/viz_Map3D.rviz', shell=True)

for ri, num_gf in enumerate(Number_GF_List):
    
    Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        # mkdir for pose traj
        cmd_mkdir = 'mkdir -p ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print(bcolors.ALERT + "====================================================================" + bcolors.ENDC)

            SeqName = SeqNameList[sn] # + '_blur_5'
            print(bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName)

            # File_Setting = './Examples/Stereo/EuRoC.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk400.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk600.yaml'
            File_Setting = config_path + '/EuRoC_yaml/EuRoC_stereo_lmk800.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk1000.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk1500.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk2000.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk2500.yaml'

            # File_Vocab   = '../ORB_Data/ORBvoc.txt'
            File_Vocab   = config_path + '/ORBvoc.bin'
            File_rosbag  = '/media/qzj/Document/grow/research/slamDataSet/EuRoC/BagFiles/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName
            # mkdir for File_traj
            cmd_mkdir = 'mkdir -p ' + File_traj
            subprocess.call(cmd_mkdir, shell=True)

            sourceRos = 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/media/qzj/Software/code//ORB_SLAM2/Examples/ROS && '
            # do viz
            cmd_slam   = str(sourceRos + 'rosrun gf_orb_slam2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)) + ' false true /cam0/image_raw /cam1/image_raw ' + File_traj)
            # no viz
            # cmd_slam   = str('rosrun gf_orb_slam2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)) + ' false false /cam0/image_raw /cam1/image_raw ' + File_traj)
            cmd_rosbag = 'rosbag play ' + File_rosbag # + ' -r 0.3' # + ' -u 20' 

            print(bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC)
            print(bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC)

            print(bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC)
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print(bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC)
            time.sleep(SleepTime)

            print(bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC)
            # 开启一个线程,并等待
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print(bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC)
            subprocess.call('rosnode kill Stereo', shell=True)
            time.sleep(SleepTime)
            # print(bcolors.OKGREEN + "Saving the map to file system" + bcolors.ENDC
            # time.sleep(15)
            subprocess.call('pkill Stereo', shell=True)

