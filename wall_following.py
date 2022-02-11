#! /usr/bin/env python
# -*- coding: utf-8 -*
# https://linklab-uva.github.io/autonomousracing/assets/files/assgn4-print.pdf
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

#获取激光雷达测量的射线距离
FILTER_VALUE = 10.0
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis

def wall_following_callback(data):
    
    # 1、计算公式：根据雷达信息，得到离墙距离AB
    THETA = np.pi / 180 * 60
    b = get_range(data, -45)
    a = get_range(data, -45 + np.rad2deg(THETA))
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))
    AB = b * np.cos(alpha)

    # 2、用PID思想来调整，保持AB为1
    if AB > 1.0:
        steering_angle = -0.1
    else:
        steering_angle = 0.1

    speed = 1

    # 3、把速度、转向角发布出去
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle=steering_angle 
    drive_msg.drive.speed=speed
    drive_pub.publish(drive_msg)

if __name__ == '__main__': 
  try:
    rospy.init_node("wall_following")
    scan_sub = rospy.Subscriber('/scan', LaserScan, wall_following_callback)
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
