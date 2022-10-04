// REF code inspired by samples in 'Mastering ROS for Robotics Programming'
//     third edition, lentin joseph and jonathan cacace, copyright 2015
// REV extended and revised by github/kamccall, last revision date: 4oct2022

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "km_diff_robot_gazebo/loc_msg.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
using namespace std;

string robotName;
km_diff_robot_gazebo::loc_msg loc_data;

void locationCallback(const nav_msgs::Odometry& mssg)
{
    // get location values from incoming mssg
    loc_data.locX = mssg.pose.pose.position.x;
    loc_data.locY = mssg.pose.pose.position.y;
    loc_data.locZ = mssg.pose.pose.position.z;

    cout << fixed << setprecision(2);
    cout << "ROBOT: " << robotName
         << "  X loc: " << setw(4) << loc_data.locX
         << "  Y loc: " << setw(4) << loc_data.locY
         << "  Z loc: " << setw(4) << loc_data.locZ << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_robot_location");
    ros::NodeHandle rosNH;

    // load robot name from parameter server
    rosNH.getParam("robot_name", robotName);

    ros::Publisher location_pub = rosNH.advertise<km_diff_robot_gazebo::loc_msg>("/location", 10);
    ros::Subscriber subLocRobot = rosNH.subscribe("/odom", 10, locationCallback); // gets data from cmd_vel topic
    ros::Rate loop_rate(5);

    // int count = 0; 

    while (ros::ok())
    {
        location_pub.publish(loc_data);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}