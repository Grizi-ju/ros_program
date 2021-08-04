#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_rectangle");    //初始化ROS节点
    ros::NodeHandle nh;          //创建节点句柄，实例化一个对象
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>
                         ("turtle1/cmd_vel", 1000);       //创建一个发布者，消息类型、话题名称以及队列长度
    srand(time(0));
    ros::Rate rate(2);   //设置循环频率，与下面的rate.sleep();配合
    int iterator = 0;

    while(ros::ok())
    {
        geometry_msgs::Twist msg;      
        msg.linear.x = 1;
        iterator++;  //迭代器++

        if(iterator==5)
        {
            iterator = 0;
            msg.linear.x = 0;
            msg.angular.z = PI;
        }
        //海龟先以x轴以线速度1进行移动，迭代到5次时，x轴线速度为0,海龟绕z轴转动90度，并循环执行

        pub.publish(msg);

        //发布消息
        ROS_INFO_STREAM("Sending random velocity command: "
                        << "linear = " << msg.linear.x 
                        << " angular = " << msg.angular.z);
        //按照循环频率延时
        rate.sleep();
    }
}
