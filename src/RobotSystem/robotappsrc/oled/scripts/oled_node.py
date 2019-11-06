#!/usr/bin/env python

import time

import Adafruit_SSD1306

from std_msgs.msg import Int8
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from jetbot.utils.utils import get_ip_address

import subprocess
import ros
import rospy
from std_msgs.msg import String

WANDER = 0
TRACE=1
OBJECTFOLLOW=2
REMOTE=3
STOP=4
class Oled():
 def __init__(self):
        rospy.init_node("oled",anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.cnt = 0   #total time
        
        self.start = time.time()
        self.errorCode = "OK"
        self.currentMode="Stop"
        self.modeState = 4;
        ##subscriber
        rospy.Subscriber('error_code',String,self.error_code_callback,queue_size=10)
        rospy.Subscriber('change_mode',Int8,self.current_mode_callback,queue_size=10)
        self.disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=2, gpio=1) # setting gpio to 1 is hack to avoid platform detection
        # Initialize library.
        self.disp.begin()
        # Clear display.
        self.disp.clear()
        self.disp.display()
        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))
        
        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0,0,self.width,self.height), outline=0, fill=0)
        # Draw some shapes.
        # First define some constants to allow easy resizing of shapes.
        self.padding = -2
        self.top = self.padding
        self.bottom = self.height-self.padding
        # Move left to right keeping track of the current x position for drawing shapes.
        self.x = 0
        # Load default font.
        self.font = ImageFont.load_default()
        rospy.loginfo("1111111111111")
        self.operate()	
        ##subscriber
        #rospy.Subscriber('error_code',String,self.error_code_callback,queue_size=10)
        #rospy.spin()
 def current_mode_callback(self,mode):
     rospy.loginfo(rospy.get_caller_id() + "i heard %d" +str( mode.data))
     self.modeState = mode.data
     if self.modeState == WANDER:
        self.currentMode = "free drive"
     elif self.modeState == TRACE:
        self.currentMode = "trace"
     elif self.modeState == OBJECTFOLLOW:
        self.currentMode = "object follow"
     elif self.modeState == REMOTE:
        self.currentMode = "remote control"
     elif self.modeState == STOP:
        self.currentMode = "stop" 
 def error_code_callback(self,data):
    print("calback")
    rospy.loginfo(rospy.get_caller_id() + "i heard %s" + data.data)
    self.errorCode = data.data

 def shutdown(self):
    rospy.loginfo("stoping oled nod")
    rospy.sleep(1)
 def operate(self):
#    while  not rospy.is_shutdown():
    rospy.loginfo("222222")
    while  not rospy.is_shutdown():
         # Draw a black filled box to clear the image.
        self.draw.rectangle((0,0,self.width,self.height), outline=0, fill=0)
    
        # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
        cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
        CPU = subprocess.check_output(cmd, shell = True )
        cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
        MemUsage = subprocess.check_output(cmd, shell = True )
        cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
        Disk = subprocess.check_output(cmd, shell = True )
        self.draw.text((self.x,self.top),       "eth0: " + str(get_ip_address('eth0')),  font=self.font, fill=255)
        #self.draw.text((self.x, self.top+8),     "usb0: " + str(get_ip_address('usb0')), font=self.font, fill=255)
        self.draw.text((self.x, self.top+8),     "mode: " + str(self.currentMode), font=self.font, fill=255)
        ##total all time
        self.draw.text((self.x, self.top+16),    "CT:"+str(int(self.cnt)),  font=self.font, fill=255)
        self.draw.text((self.x, self.top+25),    "err_code:"+self.errorCode,  font=self.font, fill=255)
	#self.draw.text((self.x, self.top+16),    str(MemUsage.decode('utf-8')),  self.font=font, fill=255)
       # self.draw.text((self.x, self.top+25),    str(Disk.decode('utf-8')),  self.font=font, fill=255)
        # Display image.
        self.disp.image(self.image)
        self.disp.display()
        time.sleep(1)
        self.cnt = time.time() - self.start

def error_code_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "i heard %s" + data.data)
    errorCode = data.data
    print('callbak',errorCode)
    
if __name__ == '__main__':
    try:
        Oled()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("error oled")
