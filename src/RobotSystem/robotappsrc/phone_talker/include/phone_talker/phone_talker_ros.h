#ifndef _PHONE_TALKER_ROS_H_
#define _PHONE_TALKER_ROS_H_
#include "phone_talker_manager.h"
//  #include "robot2_msgs/goals.h"
//  #include "robot2_msgs/goal.h"
#include <stdio.h>
#include <fstream>
#include<iostream>
#include <string>
//  #include <stdio.h>
#include "document.h"
#include "writer.h"
#include "stringbuffer.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
class CPhoneTalkerManager;
class CPhoneTalkerRos {
public:
        CPhoneTalkerRos();
        void setPTM(CPhoneTalkerManager *pPTM);
        // void talk2Ros(std::string content);
        void talk2Ros_Job(std::string content);
        void talk2Ros_Op(std::string content);
        void talk2Ros_Remote(std::string content);
        // void callback();
        void callback_error_code(const std_msgs::String::ConstPtr& codeMsg);
        int getCurrentMode();
        ros::Publisher m_Pub_mode;

private:
        void talk2Tcp(std::string frame, std::string content);
        CPhoneTalkerManager *m_pPTM;
        FILE *fp;
        int length = 0;
        std::fstream fs;
        ros::NodeHandle m_NH;
        // ros::Subsriber m_Sub;
        ros::Subscriber m_Sub_pose;
        ros::Subscriber m_Sub_jobDone;
        ros::Subscriber m_Sub_jobAccept;
        ros::Subscriber m_Sub_remoteCtrlPermission;
        ros::Subscriber m_Sub_respond;
        ros::Publisher m_Pub_job;
        ros::Publisher m_Pub_op;
        ros::Publisher m_Pub_remote;
        int currentMode = 4;
        std::string modeScene;
};
#endif  //  _PHONE_TALKER_ROS_H_
