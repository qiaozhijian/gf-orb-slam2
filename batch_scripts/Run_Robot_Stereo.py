# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# SeqNameList = {'01':'2020-07-26-19-47-34'};
SeqNameList = {'02':'2020-07-26-19-49-21'};
# SeqNameList = {'07':'2020-08-12-16-41-28'};
# SeqNameList = {'08':'2020-08-12-16-47-23'};

Result_root = '/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/GF_ORB2_Stereo_Baseline/'

Number_GF_List = [800]; # [400, 600, 800, 1000, 1500, 2000]; #
Num_Repeating = 1 # 10 # 20 #  5 # 
SleepTime = 2 # 10 # 25

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
# time.sleep(2)

# subprocess.Popen('rosrun rviz rviz -d /media/qzj/Software/code/catkin_ws/src/gf_orb_slam2/rviz/viz_Map3D.rviz', shell=True)

for ri, num_gf in enumerate(Number_GF_List):
    
    Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        # mkdir for pose traj
        cmd_mkdir = 'mkdir -p ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for key, value in SeqNameList.items():
            
            print(bcolors.ALERT + "====================================================================" + bcolors.ENDC)

            SeqName = key # + '_blur_5'
            print(bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName)

            File_Setting = config_path + '/SweepRobot_yaml/Robot_stereo_lmk' + str(num_gf) + '.yaml'

            # File_Vocab   = '../ORB_Data/ORBvoc.txt'
            File_Vocab   = config_path + '/ORBvoc.bin'
            File_rosbag  = '/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/' + SeqName + '/' + value + '.bag'
            File_traj = Experiment_dir + '/' + SeqName + '/'
            # mkdir for File_traj
            cmd_mkdir = 'mkdir -p ' + File_traj
            File_traj = File_traj + SeqName
            subprocess.call(cmd_mkdir, shell=True)

            #还是不能通过clion直接运行,只能在命令行里运行
            sourceRos = "/bin/bash -c 'source /media/qzj/Software/code/catkin_ws/devel/setup.bash' && "
            # do viz
            cmd_slam   = str(sourceRos + 'rosrun gf_orb_slam2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)) + ' false true /cam0/image_raw /cam1/image_raw ' + File_traj)
            # no viz
            # cmd_slam   = str('rosrun gf_orb_slam2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)) + ' false false /cam0/image_raw /cam1/image_raw ' + File_traj)
            cmd_rosbag = 'rosbag play --pause --clock ' + File_rosbag # + ' -r 0.3' # + ' -u 20'

            print(bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC)
            print(bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC)

            print(bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC)
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print(bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC)
            time.sleep(SleepTime)

            print(bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC)
            # 开启一个线程,并等待
            # proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print(bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC)
            subprocess.call('rosnode kill Stereo', shell=True)
            time.sleep(SleepTime)
            # print(bcolors.OKGREEN + "Saving the map to file system" + bcolors.ENDC
            # time.sleep(15)
            subprocess.call('pkill Stereo', shell=True)

