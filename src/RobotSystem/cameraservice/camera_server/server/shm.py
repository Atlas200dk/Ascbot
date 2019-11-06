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
if shmid < 0:  
    print ("System not infected")  
else:   
    shm_addr = shmat(shmid, None, 0)


def close_shm():
    shmdt(shm_addr);
    return



while True:
    print_hex(bytearray(string_at(shm_addr,4)))#in_init
    print_hex(bytearray(string_at(shm_addr+4,4)))#in_angle
    print_hex(bytearray(string_at(shm_addr+8,4)))#i_x
    print_hex(bytearray(string_at(shm_addr+12,4)))#in_y
    # test = bytearray(string_at(shm_addr+12,4))
    # print(type(string_at(shm_addr+16,4)))
    # print(type(bytearray(string_at(shm_addr+16,4))))#in_collisionStatus
    # print(struct.unpack('i', string_at(shm_addr,4)))
    value =  struct.unpack('iiiiiiii', string_at(shm_addr,32))
    print(value[0],value[1])

    memset(shm_addr+20,0,1)#out_roadfollowingSwitch
    memset(shm_addr+24,0,1)#out_collisionSwitch
    memset(shm_addr+28,0,1)#out_objectDetectionSwitch
    
    time.sleep(1)








