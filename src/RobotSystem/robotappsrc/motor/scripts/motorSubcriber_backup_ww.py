#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from jetbot import Robot
from motor.msg import MotorSpeed
from std_msgs.msg import Int16
from std_msgs.msg import Int8
import time 
import threading 
from motor.msg import LaneLine

#ww add <--
import platform
from ctypes import *
# import ctypes
import binascii
import codecs
import mmap
import sys
import time
import struct
#ww add -->


#topic motor_mode   change mode like auto_drive,control_drive,find_drive_line,find_drive_line is default;

AUTO_DRIVE = 0
CONTROL_DRIVE = 1# 
FIND_DRIVE_LINE = 2#default
#topic angle_control_speed  is find_drive_line, angle and scerity

#topic motor_cmd is remote control

#drop_control is protect drop
DropTrue = 0
DropFalse= 1

##zhuan wan de su du is 0.2
cornerSpeed = 0.2
#find drive line
driverLeftSpeed = 0.1
dirverRightSpeed = 0.1

##commonDirveSpeed30=0.3
#commonDirveSpeed25=0.25
#commonDirveSpeed20=0.20
#commonDirveSpeed15=0.15
#commonDirveSpeed10=0.1
commonDirveSpeed30=0.5
commonDirveSpeed25=0.45
commonDirveSpeed20=0.40
commonDirveSpeed15=0.35
commonDirveSpeed10=0.3

#angle devided
angle_p5=5
angle_p10=10
angle_p15=15
angle_p30=30
angle_p50=50
angle_p70=70
angle_p75=75
angle_p90=90

angle_n5=-5
angle_n10=-10
angle_n15=-15
angle_n30=-30
angle_n50=-50
angle_n70=-70
angle_n75=-75
angle_n90=-90

#ww add <..
REMOTE=0
WANDER=1
TRACE=2

TIME_STEP_WANDER=0.030
TIME_STEP_TRACE=0.030

nState = 0# should be REMOTE
#ww add ..>

#timer = None
#robot
def callback(data):
    #motors = data.data
    rospy.loginfo(rospy.get_caller_id() + "i heard direction = %s,left =%2.6f,right = %2.6f",data.direction,data.leftspeed,data.rightspeed)
    #ww add <..
    # make sure the robot is ar a right state
    if nState == REMOTE :
    #ww add ..>
        #robot.left(speed=data.leftspeed)
        direction = data.direction
        if direction == 'w':
            robot.set_motors(left_speed=data.leftspeed,right_speed=data.rightspeed)
            # robot.forward(speed=data.speed)

        elif direction == 'a':
      	    robot.set_motors(left_speed=data.leftspeed,right_speed=data.rightspeed)
            #robot.left(speed=data.speed)
        elif direction == 'd':
  	    robot.set_motors(left_speed=data.leftspeed,right_speed=data.rightspeed)
	    #robot.right(speed=data.speed)
        elif direction == 'x':
	    robot.set_motors(left_speed=data.leftspeed,right_speed=data.rightspeed)
            #robot.backward(speed=data.speed)
        elif direction == 's':
            robot.stop()
        elif direction == 'remote':
	    robot.set_motors(left_speed=data.leftspeed,right_speed=data.rightspeed) # yao kong mo shi


#ww add <..
def switch_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "i heard mode %d",data)
    robot.set_motors(0, 0)
    nState = data.data
    print('recieve data')
   
    if nState==WANDER :
        # disable timer first
        #timer.shutdown()
        memset(shm_addr+20,0,0)               #out_roadfollowingSwitch
        memset(shm_addr+24,0,1)               #out_collisionSwitch
        memset(shm_addr+28,0,1)               #out_objectDetectionSwitch
        #rospy.Timer(rospy.Duration(TIME_STEP_WANDER), drop_control_motor, oneshot=False)
        timer = rospy.Timer(rospy.Duration(TIME_STEP_WANDER), ros_timer_callback, oneshot=False)
    elif nState==TRACE :
        # disable timer first
        #timer.shutdown()
        memset(shm_addr+20,0,1)               #out_roadfollowingSwitch
        memset(shm_addr+24,0,1)               #out_collisionSwitch
        memset(shm_addr+28,0,0)               #out_objectDetectionSwitch
        #rospy.Timer(rospy.Duration(TIME_STEP_TRACE), angle_control_motor, oneshot=False)
        timer = rospy.Timer(rospy.Duration(TIME_STEP_TRACE), ros_timer_callback, oneshot=False)
    elif nState == REMOTE :
        # disable timer
        timer.shutdown()
        memset(shm_addr+20,0,0)               #out_roadfollowingSwitch
        memset(shm_addr+24,0,0)               #out_collisionSwitch
        memset(shm_addr+28,0,0)               #out_objectDetectionSwitch

def ros_timer_callback(self):
    if nState == WANDER :
        drop_control_motor(self)
    elif nState == TRACE :
        angle_control_motor(self)
    #elif nState == REMOTE :
    #    # disable timer
    #    nState == 0

## detach the memory block
def close_shm():
    shmdt(shm_addr);
    return

## print
def print_hex(bytes):
    l = [hex(int(i)) for i in bytes]
    print(" ".join(l))

##angle control speed of motor (shared memory ver.)
def angle_control_motor(self):

    #print_hex(bytearray(string_at(shm_addr,4)))#in_init
    #print_hex(bytearray(string_at(shm_addr+4,4)))#in_angle
    #print_hex(bytearray(string_at(shm_addr+8,4)))#i_x
    #print_hex(bytearray(string_at(shm_addr+12,4)))#in_y

    #bytearray(string_at(shm_addr,4))      #in_init
    #bytearray(string_at(shm_addr+4,4))    #in_angle
    #bytearray(string_at(shm_addr+8,4))    #in_x
    #bytearray(string_at(shm_addr+12,4))   #in_y
    #memset(shm_addr+20,0,1)               #out_roadfollowingSwitch
    #memset(shm_addr+24,0,1)               #out_collisionSwitch
    #memset(shm_addr+28,0,1)               #out_objectDetectionSwitch

    value =  struct.unpack('iiiiiiii', string_at(shm_addr,32))
    #print(value[0],value[1])
    print(value)

    iObstacled = value[4] #bytearray(string_at(shm_addr,4))      #in_init
    angleData = value[1] #bytearray(string_at(shm_addr+4,4))    #in_angle

    if iObstacled == 0:
        ## not security
        # todo stop
        #robot.forward(commonDirveSpeed25)
        robot.set_motors(0, 0)
    else:
        ## security

        #n
        if angle_n90 <= angleData and angleData <= angle_n75 :
         #left faster ,right slower
            robot.set_motors(commonDirveSpeed30, commonDirveSpeed10)
            print('1')
        elif angle_n75 < angleData and angleData <= angle_n50 :
            robot.set_motors(commonDirveSpeed25, commonDirveSpeed10)
            print('2')
        elif angle_n50 <= angleData and angleData <= angle_n30 :
            #left faster ,right slower
            robot.set_motors(commonDirveSpeed25, commonDirveSpeed15)
            print('3')
        elif angle_n30 < angleData and angleData <= angle_n10 :
            robot.set_motors(commonDirveSpeed25, commonDirveSpeed15)
            print('4')
        elif angle_n10 < angleData and angleData <= angle_n5 :
            robot.set_motors(commonDirveSpeed20, commonDirveSpeed15)
            print('5')
        elif angle_n5 < angleData and angleData <= angle_p5 :
            robot.set_motors(commonDirveSpeed25, commonDirveSpeed25)
            print('6')

        #p
        elif angle_p5 < angleData and angleData <= angle_p10 :
            robot.set_motors(commonDirveSpeed15, commonDirveSpeed20)
            #robot.set_motors(0, 0)
            print('7')
        elif angle_p10 < angleData and angleData <= angle_p30 :
            robot.set_motors(commonDirveSpeed15, commonDirveSpeed20)
            #robot.set_motors(0, 0)
            print('8')
        elif angle_p30 < angleData and angleData <= angle_p50 :
            robot.set_motors(commonDirveSpeed15, commonDirveSpeed25)
            #robot.set_motors(0, 0)
            print('9')
        elif angle_p50 < angleData and angleData <= angle_p70 :
            robot.set_motors(commonDirveSpeed10, commonDirveSpeed25)
            #robot.set_motors(0, 0)
            print('0')
        elif angle_p70 < angleData and angleData <= angle_p90 :
            robot.set_motors(commonDirveSpeed10, commonDirveSpeed25)
            #robot.set_motors(0, 0)
            print('a')


##drop control motor callback 
def drop_control_motor(self):
    print('wandering')
    
    value =  struct.unpack('iiiiiiii', string_at(shm_addr,32))
    print(value)

    iObstacled = value[4] #bytearray(string_at(shm_addr,4))      #in_init

    if iObstacled == 0:
        robot.set_motors(0.5, 0.5)
    else:
        robot.set_motors(0.5, -0.5)

#ww add ..>

def listener():
    rospy.init_node('motor_control',anonymous=True)
    
    rospy.Subscriber('motor_mode', Int8, switch_callback)
    rospy.Subscriber('motor_cmd',MotorSpeed,callback)
    #ww add <..
    #rospy.Timer(rospy.Duration(0.030), ros_timer_callback, oneshot=False)
    #rospy.Timer(rospy.Duration(0.030), switch_callback, oneshot=False)
    #rospy.Timer(rospy.Duration(0.030), drop_control_motor, oneshot=False)
    #ww add ..>

    rospy.spin()


if __name__ == '__main__':
    robot = Robot()

    #ww add <..

    SHM_SIZE = 32  
    SHM_KEY = 123559  

    try:  
        rt = CDLL('librt.so')  
    except:  
        rt = CDLL('librt.so.1')  

    shmget = rt.shmget  
    shmget.argtypes = [c_int, c_size_t, c_int]  
    shmget.restype = c_int  

    shmat = rt.shmat  
    shmat.argtypes = [c_int, POINTER(c_void_p), c_int]  
    shmat.restype = c_void_p  

    shmdt = rt.shmdt
    shmdt.argtypes = [POINTER(c_void_p)]  
    shmdt.restype = c_int 

    shmid = shmget(SHM_KEY, SHM_SIZE, 0o777)
    print (shmid)  
    print ("Waiting for being infected...")  
    while not rospy.is_shutdown():
        shmid = shmget(SHM_KEY, SHM_SIZE, 0o777)
        if shmid >= 0:  
            shm_addr = shmat(shmid, None, 0)
            break

    print ("Infected.")  

    #nState = 0# should be REMOTE

    #ww add ..>

    listener()
