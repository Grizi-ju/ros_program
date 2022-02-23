/***
Referenced: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
       And: https://github.com/LejunJiang

Include file names : simulator_pure_pursuit.cpp(in src) 、 csv.h（in include） 、 data.csv(in src)
Running it in siumulator of F1TENTH

1、roslaunch f1tenth_simulator simulator.launch
2、rosrun tianracer_navigation simulator_pure_pursuit in your workspace

Created by Ju Yuting
***/
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <vector>
#include <iostream>

#include "csv.h"
#include "math.h"

#define LOOKAHEAD_DISTANCE 1.20   
#define KP 1.00
#define PI 3.1415927


/********************/
/* CLASS DEFINITION */
/********************/
class SubscribeAndPublish {
public:

    SubscribeAndPublish() {

        //Publishers and Subscribers
        pub_drive = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1000);
        pub_env = n_.advertise<visualization_msgs::Marker>("/env_viz", 1000);
        pub_dynamic = n_.advertise<visualization_msgs::Marker>("/dynamic_viz", 1000);
        sub_odom = n_.subscribe("/odom", 1000, &SubscribeAndPublish::callback, this);

        // Read in all the data contains 3 columns
        io::CSVReader<3> in("data.csv");       
        double x;
        double y;
        double heading;

        // Set the blue reference trajectory and publish
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.12; 
        marker.scale.y = 0.12;
        marker.color.a = 1.0;  
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        //Convert the data in the “data.csv” into markers and display it on the map
        geometry_msgs::Point points;
        while (in.read_row(x, y, heading)) {
            xes.push_back(x);
            yes.push_back(y);
            headings.push_back(heading);
            points.x = x;
            points.y = y;
            points.z = 0.0;
            marker.points.push_back(points);
        }
    }

    void callback(const nav_msgs::Odometry& odometry_info) {

        // The calculation formula can be explained for reference : https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
        x_current = odometry_info.pose.pose.position.x;
        y_current = odometry_info.pose.pose.position.y;
        double siny_cosp = 2.0 * (odometry_info.pose.pose.orientation.w * odometry_info.pose.pose.orientation.z + odometry_info.pose.pose.orientation.x * odometry_info.pose.pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (odometry_info.pose.pose.orientation.y * odometry_info.pose.pose.orientation.y + odometry_info.pose.pose.orientation.z * odometry_info.pose.pose.orientation.z);
        heading_current = std::atan2(siny_cosp, cosy_cosp);

        if (!flag) {
            double shortest_distance = 100.0;
            for (unsigned int i = 0; i < xes.size(); i++) {
                if ((xes[i] - x_current) * (xes[i] - x_current) + (yes[i] - y_current) * (yes[i] - y_current) < shortest_distance) {
                    shortest_distance = (xes[i] - x_current) * (xes[i] - x_current) + (yes[i] - y_current) * (yes[i] - y_current);
                    current_indx = i;
                }
            }
            flag = true;
        }
        while (std::sqrt((xes[current_indx] - x_current) * (xes[current_indx] - x_current) + (yes[current_indx] - y_current) * (yes[current_indx] - y_current)) < LOOKAHEAD_DISTANCE) {
            current_indx++;
            if (current_indx > xes.size() - 1) {
                current_indx = 0;
            }
        }
        double real_distance = std::sqrt((xes[current_indx] - x_current) * (xes[current_indx] - x_current) + (yes[current_indx] - y_current) * (yes[current_indx] - y_current));
        double lookahead_angle = std::atan2(yes[current_indx] - y_current, xes[current_indx] - x_current);
        double del_y = real_distance * std::sin(lookahead_angle - heading_current);
        angle = KP * 2.00 * del_y / (real_distance * real_distance);

        // Set the red dynamic preview point marker and publish
        geometry_msgs::Point points;
        visualization_msgs::Marker marker_dy;
        points.x = xes[current_indx];
        points.y = yes[current_indx];
        points.z = 0.0;

        marker_dy.points.push_back(points);
        marker_dy.header.frame_id = "map";
        marker_dy.header.stamp = ros::Time();
        marker_dy.id = 0;
        marker_dy.type = visualization_msgs::Marker::POINTS;
        marker_dy.action = visualization_msgs::Marker::ADD;
        marker_dy.pose.position.x = 0.0;
        marker_dy.pose.position.y = 0.0;
        marker_dy.pose.position.z = 0.0;
        marker_dy.pose.orientation.x = 0.0;
        marker_dy.pose.orientation.y = 0.0;
        marker_dy.pose.orientation.z = 0.0;
        marker_dy.pose.orientation.w = 1.0;
        marker_dy.scale.x = 0.23;
        marker_dy.scale.y = 0.23;
        marker_dy.color.a = 1.0;  
        marker_dy.color.r = 1.0;
        marker_dy.color.g = 0.0;
        marker_dy.color.b = 0.0;

        SubscribeAndPublish::reactive_control();
        pub_env.publish(marker);
        pub_dynamic.publish(marker_dy);
    }

    //Ackerman Speed Controller，You can modify the value to make the car achieve the ideal driving effect
    void reactive_control() {
        ackermann_msgs::AckermannDriveStamped ackermann_drive_cmd;
        ackermann_drive_cmd.drive.steering_angle = angle;
        if (std::abs(angle) > 20.0 / 180.0 * PI) 
            {
            ackermann_drive_cmd.drive.speed = 3.0;  // 1.0
            } 
        else if (std::abs(angle) > 10.0 / 180.0 * PI) 
            {
            ackermann_drive_cmd.drive.speed = 4.5;  // 2.0
            } 
        else 
            {
            ackermann_drive_cmd.drive.speed = 6.0;  // 3.0
            }
        pub_drive.publish(ackermann_drive_cmd);
        // ROS_INFO_STREAM(angle);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_drive;
    ros::Publisher pub_env;
    ros::Publisher pub_dynamic;
    ros::Subscriber sub_odom;
    visualization_msgs::Marker marker;
    double angle;
    double x_current;
    double y_current;
    double heading_current;
    unsigned int current_indx;
    bool flag = false;
    std::vector<double> xes;
    std::vector<double> yes;
    std::vector<double> headings;

};  // End of class SubscribeAndPublish

int main(int argc, char** argv) {

    //Initiate ROS
    ros::init(argc, argv, "simulator_pure_pursuit");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}
