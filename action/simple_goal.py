#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# 动作通信：该例程在任一仿真环境下，执行/action_client通信，消息类型move_base_msgs/MoveBaseAction MoveBaseGoal 

import roslib
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  


def client():

    # 1、订阅move_base服务器的消息  
    rospy.init_node('simple_goal', anonymous=True)  
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
    move_base.wait_for_server(rospy.Duration(5.0))  
    rospy.loginfo("Connected to move base server")  


    # 2、目标点内容
    goal = MoveBaseGoal()  
    goal.target_pose.header.frame_id = 'map'  
    goal.target_pose.header.stamp = rospy.Time.now()  
    goal.target_pose.pose.position.x = 2.0
    goal.target_pose.pose.position.y = 1.0
    goal.target_pose.pose.orientation.w = 1.0


    # 3、将目标点发送出去 
    rospy.loginfo("Sending goal")  
    move_base.send_goal(goal)  


    # 4、五分钟时间限制 查看是否成功到达  
    finished_within_time = move_base.wait_for_result(rospy.Duration(300))       
    if not finished_within_time:  
        move_base.cancel_goal()  
        rospy.loginfo("Timed out achieving goal")  
    else:  
        state = move_base.get_state()  
        if state == GoalStatus.SUCCEEDED:  
            rospy.loginfo("Goal succeeded!")
        else:  
            rospy.loginfo("Goal failed！ ")  

if __name__ == '__main__':
    client()

