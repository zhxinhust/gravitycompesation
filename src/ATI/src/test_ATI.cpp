//
// Created by zhaoxin on 17-10-26.
//

#include "myATI.h"
#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <geometry_msgs/TwistStamped.h>

using namespace KDL;
using namespace geometry_msgs;


int main(int argc, char **argv)
{
    ros::init (argc, argv, "arm_kinematics");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher pub = node_handle.advertise<geometry_msgs::TwistStamped>("/ATI_force", 10);

    ros::Rate loop_rate(125);

    myATI forceSensor;
    forceSensor.initATI();
    forceSensor.biasmeasure();

    geometry_msgs::TwistStamped twist;
    float F[6];
    while(ros::ok())
    {
        forceSensor.forcemeasure(F);
        //for(int i = 0; i < 3; i++)
        {
            twist.twist.linear.x = F[0];
            twist.twist.linear.y = F[1];
            twist.twist.linear.z = F[2];
            twist.twist.angular.x = F[3];
            twist.twist.angular.y = F[4];
            twist.twist.angular.z = F[5];
        }
        twist.header.stamp = ros::Time::now();

        pub.publish(twist);
        loop_rate.sleep();
    }
}