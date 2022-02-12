/*
 * 服务通信：该例程在F1TENTH仿真环境下，执行/car_command服务，服务数据类型std_srvs/Trigger
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


ros::Publisher pub_drive;
bool pubCommand = false;
ackermann_msgs::AckermannDriveStamped vel_drive;

// service回调函数，输入参数req，输出参数res
bool commandCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	  pubCommand = !pubCommand;
    // 显示请求数据
    ROS_INFO("Publish turtle velocity command [%s]", pubCommand==true?"Yes":"No");
	  // 设置反馈数据
  	res.success = true;
  	res.message = "Change turtle command state!";

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_command_server");
    ros::NodeHandle n;

    // 创建一个名为/car_command的server，注册回调函数commandCallback
    ros::ServiceServer command_server = n.advertiseService("/car_command", commandCallback);

    // 创建一个Publisher，发布名为/drive的topic，阿克曼速度消息类型，队列长度10
    pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);

    // 循环等待回调函数
    ROS_INFO("Ready to receive car command.");

    // 设置循环的频率
    ros::Rate loop_rate(10);
    
    while(ros::ok())
	{
	  // 查看一次回调函数队列
     ros::spinOnce();

		// 如果为true，则发布速度指令, 否则停止
	  	if(pubCommand)
		{
			vel_drive.drive.speed = 1.0;	
			pub_drive.publish(vel_drive);
		}
  	else
		{
			vel_drive.drive.speed = 0.0;
			pub_drive.publish(vel_drive);
		}
      loop_rate.sleep();
	}

    return 0;
}
