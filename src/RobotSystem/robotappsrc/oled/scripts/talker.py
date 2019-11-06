#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

from std_msgs.msg import Int8
def talker():
    #pub = rospy.Publisher('error_code', String, queue_size=10)
    #pub = rospy.Publisher('angle_control_speed', Int16, queue_size=10)
    pub = rospy.Publisher('drop_control', Int8, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    
    #pub.publish(1)


    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
       # hello_str = "hello world %s" % rospy.get_time()
#	rospy.loginfo(hello_str)
#        pub.publish(hello_str)
#	angle = -60
#	rospy.loginfo(angle)
#        pub.publish(angle)



	pub.publish(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
	rospy.spin()
    except rospy.ROSInterruptException:
        pass

