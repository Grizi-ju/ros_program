#! /usr/bin/env python
# -*- coding: utf-8 -*
# Created by Xiaoju, for bilibili tutorials
# https://linklab-uva.github.io/autonomousracing/assets/files/assgn4-print.pdf
# 可正常运行，在原理简单的基础上，增加了PD，但参数没好好调

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

KP=1.00
KD=0.007
prev_error=0
FILTER_VALUE = 10.0
DESIRED_DISTANCE_RIGHT = 1.0

# 获取激光雷达射线测量距离
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis


def wall_following_callback(data):
    
    # 计算公式，得到error
    THETA = np.pi / 180 * 60
    b = get_range(data, -45)
    a = get_range(data, -45 + np.rad2deg(THETA))
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))
    AB = b * np.cos(alpha)

    projected_dis = AB + 1.00 * np.sin(alpha)
    error = DESIRED_DISTANCE_RIGHT - projected_dis
    print("projected_dis=",projected_dis)
    print("Error=",error)

    # PID控制
    drive_msg = AckermannDriveStamped()
    tmoment = rospy.Time.now().to_sec()
    prev_tmoment = 0.0
    del_time = tmoment - prev_tmoment
    drive_msg.drive.steering_angle = -(KP*error + KD*(error - prev_error)/del_time)
    prev_tmoment = tmoment
    if np.abs(drive_msg.drive.steering_angle) > np.pi / 180 * 20.0:
        drive_msg.drive.speed = 1.0
    elif np.abs(drive_msg.drive.steering_angle) > np.pi / 180 * 10.0:
        drive_msg.drive.speed = 2.0
    else:
        drive_msg.drive.speed = 3.0
    drive_pub.publish(drive_msg)


if __name__ == '__main__': 
  try:
    rospy.init_node("wall_following")
    scan_sub = rospy.Subscriber('/scan', LaserScan, wall_following_callback)
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
