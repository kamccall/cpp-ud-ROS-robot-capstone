// REF code inspired by samples in 'Mastering ROS for Robotics Programming'
//     third edition, lentin joseph and jonathan cacace, copyright 2015
// REV extended and revised by github/kamccall, last revision date: 4oct2022

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
using namespace std;

float linearX, angularZ;         // will obtain through subscription to cmd_vel topic
string robotName;                // used for robot odometry display

// BEGIN RobotSim class definition
class RobotSim
{
private:
    string _name;
    float  _linearX;
    float  _angularZ;
    float  _maxLinear;    // not used yet: could be used for future warning message(s)
    float  _maxAngular;   // not used yet: could be used for future warning message(s)
    
public:
    // setter and getter for _name private variable
    // setter: sets private variable _name (for display)
    //         sets robot_name parameter in ROS parameter server (for other nodes)
    void setName(const ros::NodeHandle& nh)
    {
        string userInput;
        cout << "enter name of your robot: ";
        getline(cin, userInput);
        _name = userInput;
        nh.setParam("robot_name", _name);    // publish robot name for other nodes
    }

    // getter: gets robot name directly from variable variable
    string getName() const
    {
        return _name;
    }

    // setter and getters for _maxLinear and _maxAngular private variables
    // setter: sets private variables _maxLinear and _maxAngular (for future use)
    //         sets max_speed_linear and max_speed_angular paramsters in ROS parameter server 
    void setSpeeds(const ros::NodeHandle& nh)
    {
        ifstream robotConfigFile("/home/kevin/catkin_ws/src/km_diff_robot_gazebo/config/robot_config.yaml");
        if (robotConfigFile)
        {
            string line;
            size_t foundMaxLinear = 0, foundMaxAngular = 0;

            while (getline(robotConfigFile, line))
            {
                foundMaxLinear  = line.find("max_vehicle_speed");
                foundMaxAngular = line.find("max_turn_speed");

                if (foundMaxLinear != string::npos)
                {
                    _maxLinear = stof(line.substr(foundMaxLinear+19, 3));
                    nh.setParam("max_speed_linear", _maxLinear);           // publish for other nodes
                }
                else if (foundMaxAngular != string::npos)
                {
                    _maxAngular = stof(line.substr(foundMaxAngular+16, 3));
                    nh.setParam("max_speed_angular", _maxAngular);         // publish for other nodes
                }
            }
        }    
    }

    // getter: get max forward (linear) speed from private variable
    float getMaxLinear() const
    {
        return _maxLinear;
    }

    // getter: get max angular speed from private variable
    float getMaxAngular() const
    {
        return _maxAngular;
    }
};
// END RobotSim class definition

// callback function to display robot odometry at defined control frequency
void odomCallback(const geometry_msgs::Twist& mssg)
{
    // pull odometry data out of mssg.data payload
    linearX  = mssg.linear.x;
    angularZ = mssg.angular.z;
    
    cout << fixed << setprecision(3);    
    cout << "ROBOT: " << robotName
         << " linear velocity: "  << setw(4) << linearX 
         << " angular velocity: " << setw(4) << angularZ << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_robot_odometry");
    ros::NodeHandle rosNH;

    RobotSim testRobot;
    testRobot.setName(rosNH);         // load robot name from console
    robotName = testRobot.getName();  // load robot name into local var for display
    testRobot.setSpeeds(rosNH);       // load robot speeds from file

    // teleop program publishes Twist messages to /cmd_vel topic
    ros::Subscriber subOdomRobot = rosNH.subscribe("/cmd_vel", 10, odomCallback); // gets data from cmd_vel topic
    ros::spin();
    return 0;
}