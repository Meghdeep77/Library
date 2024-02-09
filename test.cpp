
#include <gnc_functions.hpp>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/TwistStamped.h>

bool object_ahead  = false;
bool object_left = false;
int obstacle_angle;
double lb_dist;
double fdist;
double ow;
double ox;
double oy;
double oz;
double angle;
double siny_cosp;
double cosy_cosp;
double yaw;
double yawd = yaw * 180 / 3.1415;
//include API 

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ow = msg->orientation.w;
    ox = msg->orientation.x;
    oy = msg->orientation.y;
    oz = msg->orientation.z;
    siny_cosp = 2 * (ow * oz + ox * oy);
    cosy_cosp = 1 - 2 * (oy * oy + oz * oz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    yawd = yaw * 180 / 3.1415;
    if(yawd < 0){
    yawd = 360 + yawd;
   }
   ROS_INFO("Yaw = %f", yawd);
   
   }
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    geometry_msgs::Point current_pos = get_current_location();
	ROS_INFO("In laser callback");
	for(int i = 320; i< 400;i++){

            obstacle_angle = i/4;
            fdist = msg-> ranges[i];
            if(fdist<1){
            object_ahead  = true;
			///ROS_INFO("Obstacle ahead");
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
			geometry_msgs::TwistStamped cmd_vel_msg;
    
		
			break;

            
            }
            else{
            object_ahead  = false;

           }
           }
	for(int i = 680; i< 720;i++){

            obstacle_angle = i/4;
            lb_dist = msg-> ranges[i];
            if(lb_dist<30){
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(lb_dist<1){
            object_left = true;
			///ROS_INFO("Obstacle left");
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);

            break;
            }
            else{
            object_left = false;

           }
           
           
               }
               
               }
			   
	
           
           
               }


int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	
	//initialize control publisher/subscribers
	
	init_publisher_subscriber(gnc_node);
	ros::Subscriber lasersub = gnc_node.subscribe("/scan", 1000, laserCallback);
	ros::Subscriber imusub = gnc_node.subscribe("/mavros/imu/data", 1000, imuCallback);
	ros::Publisher velPub = gnc_node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Publisher cmd_vel_pub = gnc_node.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
	
  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(1.5);


	


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	while(ros::ok())
	{	
		
		geometry_msgs::Point current_pos = get_current_location();
	ROS_INFO("X = %f, Y = %f, Z= %f ",current_pos.x,current_pos.y,current_pos.z);
	ROS_INFO("Current Heading = %f",get_current_heading());
	angle = atan2(current_pos.y - 0 ,current_pos.x - 3)* 180 / 3.1415;
	ROS_INFO("Desired angle = %f",angle);
	geometry_msgs::TwistStamped cmd_vel_msg;

	if(!object_ahead){

		if(70< yawd && yawd< 110){
		
	 cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.5;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);}

	else if ((yawd > 0 && yawd< 30 )||(yawd < 360 && yawd > 330)){
		cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.5;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);


	}
	else if (240 < yawd && yawd < 300){
		cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = -0.5;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);


	}

	else if (150 < yawd && yawd< 210){
		cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = -0.5;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);


	}

	else{
		cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = -0.1; 
    cmd_vel_pub.publish(cmd_vel_msg);

	}
	}
	if(object_ahead){
	
		ROS_INFO("Obstacle ahead");
		cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = -0.1; 
    cmd_vel_pub.publish(cmd_vel_msg);
	ros::spinOnce();

		
    
	
	}
	if(object_left && !object_ahead){
		ROS_INFO("Obstacle Left");
		cmd_vel_msg.header.stamp = ros::Time::now();
	if(70< yawd && yawd< 110){
		
	 cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.5;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);}

	else if ((yawd > 0 && yawd< 30 )||(yawd < 360 && yawd > 330)){
		cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.5;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);


	}
	else if (240 < yawd && yawd < 300){
		cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = -0.5;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);


	}

	else if (150 < yawd && yawd< 210){
		cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = -0.5;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);


	}

	else{
		cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = -0.1; 
    cmd_vel_pub.publish(cmd_vel_msg);

	}
		}
		
	
		
		rate.sleep();
		
		ros::spinOnce();
	}
	return 0;
}