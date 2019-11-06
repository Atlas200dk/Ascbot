#!/usr/bin/env python  
 # -*- coding: utf-8 -*-  
 #  
 # This script dumps the content of a shared memory block  
 # used by Linux/Cdorked.A into a file named httpd_cdorked_config.bin  
 # when the machine is infected.  
 #  
 # Some of the data is encrypted. If your server is infected and you  
 # would like to help, please send the httpd_cdorked_config.bin  
 # to our lab for analysis. Thanks!  
 #  
 # Marc-Etienne M.Léveillé <leve...@eset.com>  
 #  
   
import platform
from ctypes import *
# import ctypes
import binascii
import codecs
import mmap
import sys
import time
import struct

# if platform.system() == 'Windows':
#     libc = cdll.LoadLibrary('msvcrt.dll')
# elif platform.system() == 'Linux':
#     libc = cdll.LoadLibrary('libc.so.6')

def print_hex(bytes):
  l = [hex(int(i)) for i in bytes]
  print(" ".join(l))

SHM_SIZE = 304  
SHM_KEY = 123559  

try:  
    rt = CDLL('librt.so')  
except:  
    rt = CDLL('librt.so.1')  
shm_addr=0
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
if shmid < 0:  
    print ("System not infected")  
else:   
    shm_addr = shmat(shmid, None, 0)


def close_shm():
    shmdt(shm_addr);
    return



while True:
    # print_hex(bytearray(string_at(shm_addr,4)))#in_init
    init_runstatus =  struct.unpack('IIII', string_at(shm_addr+0*4,4))
    print(init_runstatus)
    memset(shm_addr+4*4,1,1)#out_roadfollowingSwitch
    memset(shm_addr+5*4,0,1)#out_collisionSwitch
    memset(shm_addr+6*4,0,1)#out_objectDetectionSwitch
    memset(shm_addr+7*4,1,1)#out_roadobjectDetectionSwitch

    switch =  struct.unpack('iiii', string_at(shm_addr+4*4,4*4))
    print(switch)
    
    changeStatus =  struct.unpack('iiii', string_at(shm_addr+8*4,4*4))
    print(changeStatus)

    angle_x_y =  struct.unpack('iii', string_at(shm_addr+12*4,3*4))
    print(angle_x_y)

    collisionStatus =  struct.unpack('i', string_at(shm_addr+15*4,4))
    print(collisionStatus)

    motor_angle_RunStatus =  struct.unpack('ii', string_at(shm_addr+16*4,2*4))
    print(motor_angle_RunStatus)

    objNum = struct.unpack('i', string_at(shm_addr+18*4,4))
    print(objNum)
    if objNum[0] >=1 :
        for i in range(0,objNum[0]):
            print("index:%d " % i)
            objectData =  struct.unpack('iiiiif', string_at(shm_addr+19*4+i*24,24))
            print(objectData)  

    time.sleep(1)


#define OBJECT_RECOGNITION_NUM 10

# typedef struct object_recognition {
#   int x;
#   int y;
#   int width;
#   int height;
#   int lable;
#   float confidence;
# }S_OBJECT_RECOGNITION;


# typedef struct motor_status {
#   unsigned int init;//in
#   int roadfollowingSwitch;//out
#   int collisionSwitch;    //out
#   int objectDetectionSwitch;//out
#   int roadobjectDetectionSwitch;//out
#   int roadfollowingChange;
#   int collisionChange;
#   int objectDetectionChange;
#   int roadObjectDetectionChange;
#   int angle;//in
#   int x;//in
#   int y;//in
#   int collisionStatus;    //in
#   int motorAngle;//in
#   int motorRunStatus;//in
#   int objNum;//in
#   S_OBJECT_RECOGNITION object[OBJECT_RECOGNITION_NUM];
# }S_MOTRO_STATUS;






