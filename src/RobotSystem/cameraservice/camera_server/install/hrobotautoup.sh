#!/bin/sh


ps -ax|grep "rosmaster"|grep -v 'grep --color=auto'|awk '{print $1}'|xargs -i kill -9 {}
ps -ax|grep "motorSubcriber.py"|grep -v 'grep --color=auto'|awk '{print $1}'|xargs -i kill -9 {}
ps -ax|grep "rostopic"|grep -v 'grep --color=auto'|awk '{print $1}'|xargs -i kill -9 {}
ps -ax|grep "rtsp-server"|grep -v 'grep --color=auto'|awk '{print $1}'|xargs -i kill -9 {}
ps -ax|grep "camera_server"|grep -v 'grep --color=auto'|awk '{print $1}'|xargs -i kill -9 {}

/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace/cameraservice/rtsp-server > /dev/null &
sleep 1 
/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace/cameraservice/camera_server > /dev/null &
#cd /home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace/cameraservice/
#./bak.camera_server_debug > /dev/null &
#sleep 1
/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace/cameraservice/shm.py > /dev/null &



source /opt/ros/kinetic/setup.bash
source /root/home/catkin_ws/devel/setup.bash


export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
roslaunch phone_talker robot_up.launch > /dev/null &





