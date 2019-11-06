#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from roslog.msg import Hrobotlog
import os
import time
import threading
import numpy as np
import signal
from enum import Enum

class Level(Enum):
    CRASH = 0
    ERROR = 1
    WARN =2
    INFO = 3
    DEBUG = 4
    TRACE = 5

class Log:
    def __init__(self, fileName='testlog.txt', level=Level['INFO']):
       self.level=level
       self.fileName = fileName
       self.totalSize = 52428800  #52428800
       self.currentPosition = 0
       self.msg = ''
       if self.fileName == None :
            raise Exception("init log construct failed, fileName null")
       self.fileName = fileName
       #if self.CRASH <= level  and level <= self.TRACE :
       #    self.level=level
       #else:
       #    self.level=self.WARN
       self.fileLog = open(self.fileName, 'w')
       self.lock = threading.Lock()

    def setLevel(self ,level):
       if 0 <= level  and level <= 5 :
           self.level=level
       else:
           self.level=Level['WARN']

    def getSize(self):
#       self.fileLog = unicode(self.fileLog,'utf8')
       fsize = os.path.getsize(self.fileLog)
       fsize = fsize/float(1024*1024)
       return round(fsize, 2)

    def closeFile(self):
       self.fileLog.flush()
       self.fileLOg.close()
    def signal_handler(self,signal,frame):
        print("you press ctrl+C")
        self.closeFile()
    def writeData(self,tim,tag,function,line,level,description):
       self.lock.acquire()
       if level > self.level or level < 0 :
            #msg = time.strftime('%Y-%m-%d %H:%M:%S.%f',time.localtime(tim))[:-8]
            #msg = msg + ' ' + tag + ' ' + str(function) + '  ' + 'line['+str(line)+ ']' +'  ' + str(level) + '  ' + description + "\r\n"
            self.msg = str(tim) + ' ' + tag + ' ' + str(function) + '  ' + 'line['+str(line)+ ']' +'  ' + str(Level(2)) + '  ' + description + "\r\n"
#            self.fileLog.write(msg)
#            self.fileLog.flush()

#            print('log ' , msg)
       else:
            #msg = time.strftime('%Y-%m-%d %H:%M:%S.%f',time.localtime(tim))[:-8]
            #msg = msg + ' ' + tag + ' ' + str(function) + '  ' + 'line['+str(line)+ ']' +'  ' + str(level) + '  ' + description + "\r\n"
             self.msg = str(tim) + ' ' + tag + ' ' + str(function) + '  ' + 'line['+str(line)+ ']' +'  ' + str(Level(level)) + '  ' + description + "\r\n"
#        print("log size0:",self.currentPosition)
       self.currentPosition = self.currentPosition + len(self.msg)
       self.fileLog.write(self.msg)
       self.fileLog.flush()
       if self.fileLog.tell() > self.totalSize :
            self.currentPosition = 0;
            self.fileLog.seek(self.currentPosition,0)
       else:
            self.fileLog.seek(self.currentPosition,1)
       self.lock.release()
    def test(self) :
         print("test")



def log_call_back(log, args):
    rospy.loginfo('test log')
    robotLog = args
    robotLog.writeData(log.time,log.tag,log.function,log.line,log.level,log.description)
def robotLogInit():
    rospy.init_node('robotlog', anonymous=True)
    robotLog = Log("robotLog.txt",2)
    rospy.Subscriber("robot_log",Hrobotlog, log_call_back,robotLog)
    signal.signal(signal.SIG_IGN, robotLog.signal_handler)
    #robotLog = Log("robotLog.txt",2)
    rospy.spin()
if __name__ == '__main__':
    try:
        robotLogInit()
    except rospy.ROSInterruptException:
        pass
