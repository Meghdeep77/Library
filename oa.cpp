#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <bits/stdc++.h>


double size;
int obstacle_angle;
double dist;
double r_dist;
bool object_ahead = false;
bool object_left = false;
bool object_right = false;
bool wall_following = false;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    size = msg -> ranges.size();
    for(int i = 90; i< 270;i++){

            obstacle_angle = i/2;
            dist = msg-> ranges[i];
            if(dist<30){
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(dist<1){
            object_ahead = true;
            wall_following = true;
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);

            break;
            }
           }
           else{
            object_ahead = false;

           }
           
               }
     for(int i = 270; i< 360;i++){

            obstacle_angle = i/2;
            r_dist = msg-> ranges[i];
            if(r_dist<30){
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(r_dist<1){
            object_left = true;
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);

            break;
            }
           }
           else{
            object_left = false;

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
        
        if(wall_following == true && object_ahead == true){
            msg.linear.x = 0;
            msg.angular.z = -0.1;
            ROS_INFO("Obstacle ahead");
            velPub.publish(msg);
        }
        else if(wall_following == true && object_left == true && object_ahead == false){
            msg.linear.x = 0.5;
            msg.angular.z = 0.1;
            ROS_INFO("Obstacle on left");
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