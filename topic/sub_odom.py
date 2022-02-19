#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def callback(data):
    rospy.loginfo(data.pose.pose.orientation.w)
       
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/tianbot_mini/odom", Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
