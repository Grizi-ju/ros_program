/*
    Created by Ju Yuting, for bilibili tutorials , ID:小巨同学zz
    https://linklab-uva.github.io/autonomousracing/assets/files/assgn4-print.pdf
*/
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "math.h"

#define KP 1.00
#define KD 0.001  
#define KI 0.005
#define DESIRED_DISTANCE_RIGHT 1.0  //AB
#define DESIRED_DISTANCE_LEFT 1.0
#define LOOK_AHEAD_DIS 1.0
#define PI 3.1415927

class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        //1、发布与订阅的话题
        drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1000);
        scan_sub = nh.subscribe("/scan", 1000, &SubscribeAndPublish::callback, this);
    }

    void callback(const sensor_msgs::LaserScan& lidar_info) {

        //2、获取激光雷达测量距离 getRange
        unsigned int b_index = (unsigned int)(floor((90.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));  //dist = data.ranges[int(index)]
        double b_angle = 90.0 / 180.0 * PI;    //两条射线之间的角度   
        double a_angle = 45.0 / 180.0 * PI; 
        unsigned int a_index;
        if (lidar_info.angle_min > 45.0 / 180.0 * PI) {
            a_angle = lidar_info.angle_min;
            a_index = 0;
        } else {
            a_index = (unsigned int)(floor((45.0 / 180.0 * PI - lidar_info.angle_min) / lidar_info.angle_increment));
        }            
        double a_range = 0.0;   
        double b_range = 0.0;
        if (!std::isinf(lidar_info.ranges[a_index]) && !std::isnan(lidar_info.ranges[a_index])) {   //isinf()是否为无穷大   ranges：转一周的测量数据一共360个，每度是一个
            a_range = lidar_info.ranges[a_index];   //得到a的长度
        } else {
            a_range = 100.0;
        }
        if (!std::isinf(lidar_info.ranges[b_index]) && !std::isnan(lidar_info.ranges[b_index])) {
            b_range = lidar_info.ranges[b_index];   //得到b的长度
        } else {
            b_range = 100.0;
        }
     
        //3、计算公式，参考pdf  
        //在车的右边得到两条射线a、b来确定车到右墙的距离AB和相对于AB的方向
        double alpha = atan((a_range * cos(b_angle - a_angle) - b_range) / (a_range * sin(b_angle - a_angle)));   //b_angle - a_angle为a、b夹角THETA
        double AB = b_range * cos(alpha);    //实际离右墙距离
        double projected_dis = AB + LOOK_AHEAD_DIS * sin(alpha);     //Due to the forward motion of the car and a finite delay in execution of the control and perception nodes; 
                                                                     //we instead, virtually project the car forward a certain distance from its current position.  
                                                                     //LOOK_AHEAD_DIS = AC = 1   projected_dis = CD
        error = DESIRED_DISTANCE_RIGHT - projected_dis;   //求出误差
        ROS_INFO("projected_dis = %f",projected_dis);
        ROS_INFO("error = %f",error);
        ROS_INFO("del_time = %f\n",del_time);
        SubscribeAndPublish::pid_control();
    }

        //4、PID控制器
    void pid_control() {
        ackermann_msgs::AckermannDriveStamped ackermann_drive_result;
        double tmoment = ros::Time::now().toSec();
        del_time = tmoment - prev_tmoment;   //当前时刻-上一个时刻 = 间隔时刻
        integral += prev_error * del_time;   //对误差积分，也就是误差的无限和  积分=积分+累计误差
        ackermann_drive_result.drive.steering_angle = -(KP * error + KD * (error - prev_error) / del_time + KI * integral);
        prev_tmoment = tmoment;    //时间的迭代

        //不同情况下的速度调整，转弯时速度降低，直行时速度加快
        if (abs(ackermann_drive_result.drive.steering_angle) > 20.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 2.0;
            //speed = 1.3;
        } else if (abs(ackermann_drive_result.drive.steering_angle) > 10.0 / 180.0 * PI) {
            ackermann_drive_result.drive.speed = 3.0;
            //speed = 2.0;
        } else {
            ackermann_drive_result.drive.speed = 5.0;
            //speed = 3.0;
        }
        drive_pub.publish(ackermann_drive_result);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher drive_pub;
    ros::Subscriber scan_sub;
    double prev_error = 0.0;    //前一个误差
    double prev_tmoment = ros::Time::now().toSec();    //当前时间，单位秒s
    double error = 0.0;
    double integral = 0.0;
    double speed = 0.0;
    double del_time = 0.0;

}; 

int main(int argc, char** argv) {

    ros::init(argc, argv, "wall_following_pid");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}
