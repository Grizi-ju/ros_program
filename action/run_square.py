#!/usr/bin/env python  
# -*- coding: utf-8 -*  
# 动作通信：该例程在room_square仿真环境下，执行/action_client通信，消息类型move_base_msgs/MoveBaseAction MoveBaseGoal

import roslib
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from tf.transformations import quaternion_from_euler  
from math import pi
  

class RunSquare():  
    def __init__(self):  
        rospy.init_node('run_square', anonymous=False)  
        rospy.on_shutdown(self.shutdown)  
        
        # 1、订阅move_base服务器消息 + 发布速度话题  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,  queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")  
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.loginfo("Connected to move base server")  
        rospy.loginfo("Starting navigation test")  
            

        # 2、目标点内容
        
        # 创建四元数列表，保存目标的角度数据  
        quaternions = list()  
 
        # 定义四个顶角处机器人的方向角度（Euler angles:http://zh.wikipedia.org/wiki/%E6%AC%A7%E6%8B%89%E8%A7%92)  
        euler_angles = (pi/2, pi, 3*pi/2, 0)  
          
        # 将上面的Euler angles转换成Quaternion的格式  
        for angle in euler_angles:  
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')  
            q = Quaternion(*q_angle)  
            quaternions.append(q)  

        # 创建四个导航点的位置（角度和坐标位置）  
        waypoints = list()  
        waypoints.append(Pose(Point(5.1, 2.0, 0.0), quaternions[0]))  
        waypoints.append(Pose(Point(1.8,5.5, 0.0), quaternions[1]))  
        waypoints.append(Pose(Point(0.0, 2, 0.0), quaternions[2]))  
        waypoints.append(Pose(Point(0.0, 0, 0.0), quaternions[3]))  
  
        for poses in range(0,3):
            i=0
            while i<4 and not rospy.is_shutdown():              
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id='map'
                goal.target_pose.header.stamp=rospy.Time.now()
                goal.target_pose.pose=waypoints[i]
                self.send_goal(goal)

                i+=1



    def send_goal(self, goal):  

            # 3、将目标点发送出去 
            rospy.loginfo("Sending goal")  
            self.move_base.send_goal(goal)  
              
            # 4、五分钟时间限制 查看是否成功到达  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                else:  
                    rospy.loginfo("Goal failed！ ")  
                   
  
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  


if __name__ == '__main__':  
    try:  
        RunSquare()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("Navigation finished.")
