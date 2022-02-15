#! /usr/bin/env python
# -*- coding: utf-8 -*
# Created by Xiaoju, for bilibili tutorials, ID:小巨同学zz
# https://linklab-uva.github.io/autonomousracing/assets/files/assgn4-print.pdf
# 可正常运行，在wall_following.py的基础上，增加了PD

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

KP=1.000
KD=0.005
prev_error=0
FILTER_VALUE = 10.0
DESIRED_DISTANCE_RIGHT = 1.0

# 获取激光雷达测量距离,可得到射线ab的长度
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis


def wall_following_callback(data):
    
    # 1、计算公式：根据雷达信息，得到error
    THETA = np.pi / 180 * 45
    a = get_range(data, 45)
    b = get_range(data, 45 + np.rad2deg(THETA))
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))
    AB = b * np.cos(alpha)

    projected_dis = AB + 1.00 * np.sin(alpha)
    error = DESIRED_DISTANCE_RIGHT - projected_dis
    print("projected_dis=",projected_dis)
    print("Error=",error)

    # 2、PID控制,公式代替条件判断
    tmoment = rospy.Time.now().to_sec()
    prev_tmoment = 0.0
    del_time = tmoment - prev_tmoment
    steering_angle = -(KP*error + KD*(error - prev_error)/del_time)
    prev_tmoment = tmoment

    if np.abs(steering_angle) > np.pi / 180 * 20.0:
        speed = 1.5
    elif np.abs(steering_angle) > np.pi / 180 * 10.0:
        speed = 2.5
    else:
        speed = 3.5

    # 3、把速度、转向角发布出去
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle = steering_angle
    drive_msg.drive.speed = speed
    drive_pub.publish(drive_msg)



if __name__ == '__main__': 
  try:
    rospy.init_node("wall_following")
    scan_sub = rospy.Subscriber('/scan', LaserScan, wall_following_callback)
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

