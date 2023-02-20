#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include <bits/stdc++.h>
using namespace std;


geometry_msgs::Vector3 pub_data;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  double ax = msg->linear_acceleration.x;
  double ay = msg->linear_acceleration.y;
  double az = msg->linear_acceleration.z;

  ROS_INFO("Linear acceleration: (%f, %f, %f)", ax, ay, az);
}


 int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;

    ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //ros::Publisher imuPub = n.advertise<geometry_msgs::Vector3>("/Imu_lin_pub", 1000);
    ros::Subscriber sub1 = n.subscribe("/imu", 1000, imuCallback);
    
    ros::Rate loop_rate(20);

    geometry_msgs::Twist msg;
    while (ros::ok())
    {
        msg.linear.x = 0.1;
        velPub.publish(msg);
        //imuPub.publish(pub_data);
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}



