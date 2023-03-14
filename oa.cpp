#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <bits/stdc++.h>


double size;
int obstacle_angle;
double dist;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    size = msg -> ranges.size();
    for(int i = 0; i< size;i++){

            obstacle_angle = i/2;
            dist = msg-> ranges[i];
            if(dist<30){
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(dist<1)
            break;


            }
           
        
        
    }
  
    
    ROS_INFO("size = %f",size);


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser");
    ros::NodeHandle n;
    ros::Subscriber lasersub = n.subscribe("/scan", 1000, laserCallback);
    ros::Rate loop_rate(20);
    ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    geometry_msgs::Twist msg;
    

    while (ros::ok())
    {   
        
        if(dist<1){
            msg.linear.x = 0;
            msg.angular.z = 0.1;
            velPub.publish(msg);
        }
        else{
            msg.linear.x = 1;
            velPub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
                        }
}