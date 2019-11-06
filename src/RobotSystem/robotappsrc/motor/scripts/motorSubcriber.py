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

import platform
from ctypes import *
# import ctypes
import binascii
import codecs
import mmap
import sys
import time
import struct
import ipywidgets
from roslog.msg import Hrobotlog
import inspect
import sys


REMOTE=3
WANDER=0
TRACE=1
OBJECTFOLLOW=2
STOP = 4
TIME_STEP_WANDER=0.030
TIME_STEP_DROPGOBACK=1.0
TIME_STEP_TRACE=0.010
firstAngle=0 
k = 0.5 
dropAngleCnt=5000
dropAngleFlag=1
angleFirst=0;
angle = 0.0000000
angle_last = 0.0000000
angleCnt=-1
runFlag=1
runStep=0
dropStep=0
dropFlag=1
stopCnt=0
shmid=-1
shm_addr = -1
class Motor():    
    pubRobotLog=None 
     
    def __init__(self):
        rospy.init_node('motor_control',anonymous=True)
        self.pubRobotLog = rospy.Publisher('robot_log', Hrobotlog,queue_size = 100)
        self.nState = REMOTE# should be REMOTE
        self.robot = Robot()
        self.SHM_SIZE = 32  
        self.SHM_KEY = 123559
        self.timer=None
        self.timerDrop=None
        self.isBack=True
        try:  
            rt = CDLL('librt.so')  
        except:  
            rt = CDLL('librt.so.1')  

        self.shmget = rt.shmget  
        self.shmget.argtypes = [c_int, c_size_t, c_int]  
        self.shmget.restype = c_int  

        self.shmat = rt.shmat  
        self.shmat.argtypes = [c_int, POINTER(c_void_p), c_int]  
        self.shmat.restype = c_void_p  
   #     shmid        
   #     self.shm_addr=0
        shmdt = rt.shmdt
        shmdt.argtypes = [POINTER(c_void_p)]  
        shmdt.restype = c_int 
	    
        self.speed_gain_slider = 0.4
        self.steering_gain_slider = 0.4#0.4
        self.speed_slider = 0.2
        self.steering_dgain_slider = 0.4#0.5
        self.steering_bias_slider = 0.01#0.01


    def callback(self, data):
#         print('remote_test')
        self.publog(self.pubRobotLog,time.time(),'motor',sys._getframe().f_code.co_name,sys._getframe().f_lineno,3,str(data.direction))
        # make sure the robot is ar a right state
        if self.nState == REMOTE :
            direction = data.direction
            if direction == 'w':
                self.robot.set_motors(left_speed=data.leftspeed,right_speed=data.rightspeed)
            elif direction == 'a':
          	self.robot.set_motors(left_speed=data.leftspeed,right_speed=data.rightspeed)
                #robot.left(speed=data.speed)
            elif direction == 'd':
      	        self.robot.set_motors(left_speed=data.leftspeed,right_speed=data.rightspeed)
    	        #robot.right(speed=data.speed)
            elif direction == 'x':
    	        self.robot.set_motors(left_speed=data.leftspeed,right_speed=data.rightspeed)
                #robot.backward(speed=data.speed)
            elif direction == 's':
                self.robot.stop()
            elif direction == 'remote':
    	        self.robot.set_motors(left_speed=data.leftspeed,right_speed=data.rightspeed) # yao kong mo shi
                print(data)
    
    def publog(self,pubRobotLog,tim,tag,function,line,level,description):
         hrobotlog = Hrobotlog()
         hrobotlog.time = tim
         hrobotlog.tag = tag
         hrobotlog.function = function
         print(sys._getframe().f_code.co_name)
         hrobotlog.line = line
         hrobotlog.level = level
         hrobotlog.description = description
         pubRobotLog.publish(hrobotlog)

        
    def switch_callback(self, data):
#        rospy.loginfo(rospy.get_caller_id() + "i heard mode %d",data)
        self.robot.set_motors(0, 0)
        self.nState = data.data
        global angleCnt
        global stopCnt
        stopCnt = 0
        angleCnt = 0
        print('change mode :' + str(data.data))
        print(type(sys._getframe().f_lineno))
              
        if self.nState==WANDER :
            # disable timer first
            global shmid
            if shmid < 0 :
                print ("Waiting for being infected...")
                while not rospy.is_shutdown():
                    shmid = self.shmget(self.SHM_KEY,self.SHM_SIZE, 0o777)
                    if shmid >= 0:
                        global shm_addr
                        shm_addr = self.shmat(shmid, None, 0)
                        break

                print ("Infected."+str(shmid))

            memset(shm_addr+1*4,0,1)           #out_roadfollowingSwitch
            memset(shm_addr+2*4,1,1)           #out_collisionSwitch
            memset(shm_addr+3*4,0,1)           #out_objectDetectionSwitch
            memset(shm_addr+4*4,1,1)           #out_roadobjectectionSwitch
            if self.timer !=None :
                try :
                    self.timer.shutdown()
                except :
                    print('Timer is unactivated.')
            self.timer = rospy.Timer(rospy.Duration(TIME_STEP_WANDER), self.ros_timer_callback, oneshot=False)
        elif self.nState==TRACE :
            # disable timer first
            global shmid
            if shmid < 0 :
                print ("Waiting for being infected...")
                while not rospy.is_shutdown():
                    shmid = self.shmget(self.SHM_KEY, self.SHM_SIZE, 0o777)
                    if shmid >= 0:
                        global shm_addr
                        shm_addr = self.shmat(shmid, None, 0)
                        break
                print ("Infected.")
 
            memset(shm_addr+1*4,1,1)           #out_roadfollowingSwitch
            memset(shm_addr+2*4,0,1)           #out_collisionSwitch
            memset(shm_addr+3*4,0,1)           #out_objectDetectionSwitch    
            memset(shm_addr+4*4,1,1)           #out_roadobjectDetecttionSwitch
            if self.timer !=None :
                try :
                    self.timer.shutdown()
                except :
                    print('Timer is unactivated.')
            self.timer = rospy.Timer(rospy.Duration(TIME_STEP_TRACE), self.ros_timer_callback, oneshot=False)
#        elif self.nState==OBJECTFOLLOW :
	    #nothing
        elif self.nState==REMOTE :
            self.robot.set_motors(0, 0)
            self.robot.stop()
            if self.timer !=None :
                try :
                    self.timer.shutdown()
                except :
                    print('Timer is unactivated.')
 	        if shmid >= 0 :
                 memset(shm_addr+1*4,0,1)  #out_roadfollowingSwitch
                 memset(shm_addr+2*4,0,1)           #out_collisionSwitch
                 memset(shm_addr+3*4,0,1)           #out_objectDetectionSwitch    
                 memset(shm_addr+4*4,0,1)           #out_roadobjectDetecttionSwitch
            
        elif self.nState==STOP :
            self.robot.set_motors(0, 0)
            self.robot.stop()
            if self.timer !=None :
                try :
                    self.timer.shutdown()
                except :
                    print('Timer is unactivated.')
            if shm_addr > 0 :
                memset(shm_addr+1*4,0,1)  #out_roadfollowingSwitch
                memset(shm_addr+2*4,0,1)           #out_collisionSwitch
                memset(shm_addr+3*4,0,1)           #out_objectDetectionSwitch    
                memset(shm_addr+4*4,0,1)           #out_roadobjectDetecttionSwitch
        self.publog(self.pubRobotLog,time.time(),'motor',sys._getframe().f_code.co_name,sys._getframe().f_lineno,3,str(data.data))     
    #wp start
    def execute(self,change):
        global angle, angle_last
        self.speed_slider = self.speed_gain_slider
    
        angle = change/100.00
        pid = angle * self.steering_gain_slider + (angle - angle_last) * self.steering_dgain_slider
        angle_last = angle
        self.publog(self.pubRobotLog,time.time(),'motor',sys._getframe().f_code.co_name,sys._getframe().f_lineno,3,'angle:' + str(angle))
#         print("paul 0 angle %10.3f",angle)
#         print("paul 1 anglelst %10.3f",angle_last)
        steering_slider = pid + self.steering_bias_slider
        self.publog(self.pubRobotLog,time.time(),'motor',sys._getframe().f_code.co_name,sys._getframe().f_lineno,3,'steering_slider:' + str(steering_slider))
        self.robot.left_motor.value = max(min(self.speed_slider+ steering_slider, 1.0), 0.000)
        self.robot.right_motor.value = max(min(self.speed_slider - steering_slider, 1.0), 0.000)
#         print("paul l speed ",self.robot.left_motor.value)
#         print("paul r speed ",self.robot.right_motor.value)
        print("paul  angle ",angle)
        self.publog(self.pubRobotLog,time.time(),'motor',sys._getframe().f_code.co_name,sys._getframe().f_lineno,3,'left speed:' + str(self.robot.left_motor.value) + 'right speed:' +  str(self.robot.right_motor.value))
	#self.robot.set_motors(l_wheel, r_wheel)

    def dropBack_timer_callback(self,timer):
        self.robot.set_motors(0.4,-0.4)
        rospy.sleep(0.5)
        self.timer = rospy.Timer(rospy.Duration(TIME_STEP_WANDER), self.ros_timer_callback, oneshot=False)
        
    #wp end
    def ros_timer_callback(self, timer):
        if self.nState == WANDER :
            self.drop_control_motor()
        elif self.nState == TRACE :
            self.angle_control_motor()
        #elif nState == REMOTE :
        #    # disable timer
        #    nState == 0
    
    ## detach the memory block
    def close_shm(self):
        shmdt(shm_addr);
        return
    
    ## print
    def print_hex(self, bytes):
        l = [hex(int(i)) for i in bytes]
        print(" ".join(l))
    
    ##angle control speed of motor (shared memory ver.)
    def angle_control_motor(self):
            
        global runStep  
        global runFlag
        memset(shm_addr+1*4,1,1)           #out_roadfollowingSwitch
        memset(shm_addr+2*4,0,1)           #out_collisionSwitch
        memset(shm_addr+3*4,0,1)           #out_objectDetectionSwitch    
        memset(shm_addr+4*4,1,1)           #out_roadobjectDetecttionSwitch             
        runStep+=1;
        if runStep >= 15 :
            runStep = 0
            global angleCnt
            init =  struct.unpack('I', string_at(shm_addr+0*4,4))
#             print("paul init",init)
#             print("paul angleCnt",angleCnt)
            if angleCnt == init :
                #stop motor
                self.robot.set_motors(0, 0)
                runFlag = 0
                
                global stopCnt
                stopCnt += 1
                self.publog(self.pubRobotLog,time.time(),'motor angle control',sys._getframe().f_code.co_name,sys._getframe().f_lineno,3,str("time out:") + str(stopCnt))

            else :
                angleCnt = init
                runFlag = 1
            
        if runFlag == 1 :
            changeStatus =  struct.unpack('iiii', string_at(shm_addr+5*4,4*4))
        #           print(changeStatus)
            angle_x_y =  struct.unpack('iii', string_at(shm_addr+9*4,3*4))
#             print(angle_x_y)
            collisionStatus =  struct.unpack('i', string_at(shm_addr+12*4,4))
#             print(collisionStatus)
            motor_angle_RunStatus =  struct.unpack('ii', string_at(shm_addr+13*4,2*4))
        #       print(motor_angle_RunStatus)
            objNum = struct.unpack('i', string_at(shm_addr+15*4,4))
        #      print(objNum)
            if objNum[0] >=1 :
                for i in range(0,objNum[0]):
        #            print("index:%d " % i)
                    objectData =  struct.unpack('iiiiif', string_at(shm_addr+16*4+i*24,24))
        #           print(objectData)  
        
            if motor_angle_RunStatus[1] == 0  :   # todo: this value has to be modified after wandering training
        ## not safe
                self.robot.set_motors(0, 0)
            else:
        #angleData=motor_angle_RunStatus[0]
                angleData = angle_x_y[0] #bytearray(string_at(shm_addr+4,4))    #in_angle
                self.execute(angleData)
   
    ##drop control motor callback 
    def drop_control_motor(self):
#         print('wandering')
        global dropStep
        global dropFlag
        global angleCnt
        memset(shm_addr+1*4,0,1)           #out_roadfollowingSwitch
        memset(shm_addr+2*4,1,1)           #out_collisionSwitch
        memset(shm_addr+3*4,0,1)           #out_objectDetectionSwitch
        memset(shm_addr+4*4,1,1)           #out_roadobjectectionSwitch
        dropStep +=1
        if dropStep > 10 :
            dropStep = 0
            index =  struct.unpack('I', string_at(shm_addr+0*4,4))
            if angleCnt == index :
                self.robot.set_motors(0, 0)
                runFlag = 0
                
                global stopCnt
                stopCnt += 1
                self.publog(self.pubRobotLog,time.time(),'motor angle control',sys._getframe().f_code.co_name,sys._getframe().f_lineno,3,str("time out:") + str(stopCnt))
            else :
                dropFlag =1
                angleCnt = index
        if dropFlag==1:
            #iObstacled = value[4] 
            changeStatus =  struct.unpack('iiii', string_at(shm_addr+5*4,4*4))
         #   print(changeStatus)
            angle_x_y =  struct.unpack('iii', string_at(shm_addr+9*4,3*4))
#             print(angle_x_y)
            collisionStatus =  struct.unpack('i', string_at(shm_addr+12*4,4))
#             print(collisionStatus)
            motor_angle_RunStatus =  struct.unpack('ii', string_at(shm_addr+13*4,2*4))
#             print(motor_angle_RunStatus)
            objNum = struct.unpack('i', string_at(shm_addr+15*4,4))
#             print(objNum)
            if objNum[0] >=1 :
               for i in range(0,objNum[0]):
#                    print("index:%d " % i)
                   objectData =  struct.unpack('iiiiif', string_at(shm_addr+16*4+i*24,24))

#             print("motor angle:",motor_angle_RunStatus[0])
            #if iObstacled == 0 or iObstacled == 1 :   # todo: this value has to be modified after wandering training
            if collisionStatus[0] == 1 :
                if motor_angle_RunStatus[0] == -90 :
                    self.robot.set_motors(-0.4, 0.4)
                elif motor_angle_RunStatus[0] == 90 :   # todo: this value has to be modified after wandering training
                # if it is not safe, whether is 
                    self.robot.set_motors(0.4, -0.4)
                else:
                    self.robot.set_motors(0.4, 0.4)
            else :
                    self.robot.set_motors(0.4, -0.4)
    
    def listener(self):
        rospy.Subscriber('change_mode', Int8, self.switch_callback)
        rospy.Subscriber('motor_cmd', MotorSpeed, self.callback)
        rospy.spin()
if __name__ == '__main__':
    #self.pubRobotLog = rospy.Publisher('robot_log', Hrobotlog,queue_size = 100)
    motor = Motor()
    motor.listener()
