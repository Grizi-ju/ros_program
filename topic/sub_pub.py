#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def callback(data):

    rospy.loginfo(data.theta)
    car_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    car_cmd = Twist()
    car_cmd.angular.z = 1.0
    car_pub.publish(car_cmd)
    if data.theta>1.0:
    	car_cmd.linear.x=0.7
    	car_pub.publish(car_cmd)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
