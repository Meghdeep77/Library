#include <gnc_functions.hpp>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/TwistStamped.h>
#include <bits/stdc++.h>


double ow;
double ox;
double oy;
double oz;
double siny_cosp;
double cosy_cosp;
double yaw;
double yawd;
double lat;
double lon;
double x ;
double y ;
double dist;
double sdist;
double angle;
int flag = 0;
int flag2 = 0;
int flag3 =0;
double so;
double start_lon;
double start_lat;
double hdist;
double size;
int obstacle_angle;
double fdist;
double l_dist;
double lb_dist;
double last_left;
double denominator;
double numerator;
double raw_yaw;
double temp;
double raw_angle;
int delta_cw;
int delta_ccw;

bool object_ahead = false;
bool object_left = false;
bool object_right = false;
bool object_back_left = false;
bool wall_following = false;
double dist_to_line(double x0, double y0);
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
    raw_yaw = yaw;
    if(yawd < 0 ){
        yawd = yawd + 360;
    }
    
   ROS_INFO("Yaw = %f", yawd);

   }

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    geometry_msgs::Point current_pos = get_current_location();
    lon = current_pos.x;
    lat = current_pos.y;
    dist = sqrt(pow(lat - y, 2) + pow(lon - x, 2));
    if(flag2 == 0){
    start_lon = current_pos.x;
    start_lat = current_pos.y;
    flag2 ++;
    sdist = 0;
    }


  sdist  = sqrt(pow(start_lat - lat, 2) + pow(start_lon - lon, 2)); 
  ROS_INFO("sdist = %f ",sdist);
	ROS_INFO("In laser callback");
    ROS_INFO("Distance to line : %f",dist_to_line(lon,lat));
    ROS_INFO("Current Coordinates : X : %f Y : %f Z: %f ",current_pos.x,current_pos.y,current_pos.z);
    ROS_INFO("Starting Coordinates : X : %f Y : %f ",start_lon,start_lat);
	for(int i = 300; i< 415;i++){
            ;
            obstacle_angle = i/2;
            fdist = msg-> ranges[i];
            if(fdist<1){
            object_ahead  = true;
            wall_following = true;
			///ROS_INFO("Obstacle ahead");
            ROS_INFO("Object front");
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
			geometry_msgs::TwistStamped cmd_vel_msg;


			break;


            }
            else{
            object_ahead  = false;

           }
           }
	for(int i = 530; i< 550 ;i++){
            
            obstacle_angle = i/2;
            lb_dist = msg-> ranges[i];
            
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(lb_dist<1.5){
            object_left = true;
			///ROS_INFO("Obstacle left");
            ROS_INFO("Object Left");
            ///ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            temp = obstacle_angle;

            break;
            }
            else{
            object_left = false;

           }


            

               }

    
			for(int i = 600; i< 700;i++){

            obstacle_angle = i/2;
            l_dist = msg-> ranges[i];
        
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(l_dist<2){
            object_back_left = true;
            ROS_INFO("Corner");
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);

            break;
            }
            else{
            object_back_left= false;

           }
           
           
               }




               }
    

double dist_to_line(double x0, double y0){

double numerator = fabs((y - start_lat) * x0 - (x - start_lon) * y0 + x * start_lat - y * start_lon);
double denominator = sqrt(pow(y - start_lat, 2) + pow(x - start_lon, 2));
return numerator/denominator;

}


int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");


	//initialize control publisher/subscribers

	init_publisher_subscriber(gnc_node);
	ros::Subscriber lasersub = gnc_node.subscribe("/scan", 1000, laserCallback);
	ros::Subscriber imusub = gnc_node.subscribe("/mavros/imu/data", 1, imuCallback);
	ros::Publisher cmd_vel_pub = gnc_node.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
     if(flag3 == 0){
    std::cout<<"Enter Longitude";
    std::cin>>x;
    std::cout<<"Enter Lattitude";
    std::cin>>y;
    ros::spinOnce();
    flag3++;
    }

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
    ROS_INFO("Ahead = %d, Left = %d, LeftBack= %d ",object_ahead,object_left,object_back_left);
	ROS_INFO("Current Heading = %f",get_current_heading());
	ROS_INFO("Dist to point  = %f",dist);
    ROS_INFO("sdist = %f ",sdist);
	geometry_msgs::TwistStamped cmd_vel_msg;
    cmd_vel_msg.header.stamp = ros::Time::now();


    if(wall_following == true){
        ROS_INFO("Wall following");
        if(object_ahead == false && object_left == false && object_back_left == false ){
        wall_following = false;
        flag2 = 0;
        flag = 0;
        sdist = 0;
       
        
        ros::spinOnce();

    }
	
    if( object_left == false && object_ahead == false && object_back_left == true){
        ROS_INFO("Corner");
        geometry_msgs::TwistStamped cmd_vel_msg;
	 cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.1;
    cmd_vel_pub.publish(cmd_vel_msg);
    ros::spinOnce();

    }
	
	if(object_ahead){

		ROS_INFO("Obstacle ahead");
        geometry_msgs::TwistStamped cmd_vel_msg;
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
        ROS_INFO("Angle : %f",temp);
        
		
	geometry_msgs::TwistStamped cmd_vel_msg;
	 cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.5 *cos(yaw);  
    cmd_vel_msg.twist.linear.y = 0.5 * sin(yaw);  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);
    ros::spinOnce();}

   
		}
        else{
         angle = atan2(y-lat,x -lon)* 180 / 3.1415;
         raw_angle = angle;
          if(angle < 0){
        angle = 360 + angle;}
        delta_cw = (int(angle) - int(yawd) + 360) % 360;
        delta_ccw = (int(yawd) - int(angle) + 360) % 360;
        ROS_INFO("Going to point");
        ROS_INFO("Desired angle = %f",angle);
        ROS_INFO("Yaw = %f",yawd);
        if(dist < 1){
         geometry_msgs::TwistStamped cmd_vel_msg;
	cmd_vel_msg.header.stamp = ros::Time::now();	
    ROS_INFO("Reached");
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);
    land();
	ros::spinOnce();
        }
        if(abs(yawd - angle) < 5 && dist > 1 && wall_following == false){
            geometry_msgs::TwistStamped cmd_vel_msg;
             ROS_INFO("Going");
        cmd_vel_msg.header.stamp = ros::Time::now();
        cmd_vel_msg.twist.linear.x = 0.5 * cos(yaw);  
        cmd_vel_msg.twist.linear.y = 0.5 * sin(yaw);  
        cmd_vel_msg.twist.linear.z = 0.0;  
        cmd_vel_msg.twist.angular.x = 0.0; 
        cmd_vel_msg.twist.angular.y = 0.0; 
        cmd_vel_msg.twist.angular.z = 0.0; 
        cmd_vel_pub.publish(cmd_vel_msg);
        ros::spinOnce();}

        else if(dist > 1 && sdist < 2 && wall_following == false && abs(yawd - angle) >= 5){
                ROS_INFO("error %f",abs(yawd - angle));
            geometry_msgs::TwistStamped cmd_vel_msg;
        if(delta_cw > delta_ccw){
            cmd_vel_msg.header.stamp = ros::Time::now();
            ROS_INFO("angle: %f yaw: %f",angle,raw_yaw);	
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = -0.1; 
    cmd_vel_pub.publish(cmd_vel_msg);
	ros::spinOnce();}

    else{
         ROS_INFO("angle: %f yaw: %f",angle,raw_yaw);	
        cmd_vel_msg.header.stamp = ros::Time::now();	
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.1; 
    cmd_vel_pub.publish(cmd_vel_msg);
	ros::spinOnce();


    }


	}
        else{
            if(abs(yawd - angle) >= 5 && sdist > 2 && wall_following == false){
                ROS_INFO("Correcting");
                if(delta_cw > delta_ccw){
                
                geometry_msgs::TwistStamped cmd_vel_msg;
        cmd_vel_msg.header.stamp = ros::Time::now();
        cmd_vel_msg.twist.linear.x = 0.5 * cos(yaw);  
        cmd_vel_msg.twist.linear.y = 0.5 * sin(yaw);  
        cmd_vel_msg.twist.linear.z = 0.0;  
        cmd_vel_msg.twist.angular.x = 0.0; 
        cmd_vel_msg.twist.angular.y = 0.0; 
        cmd_vel_msg.twist.angular.z = -0.01; 
        cmd_vel_pub.publish(cmd_vel_msg);
        ros::spinOnce();
                }

                else{
                    ROS_INFO("Correcting");

          geometry_msgs::TwistStamped cmd_vel_msg;
        cmd_vel_msg.header.stamp = ros::Time::now();
        cmd_vel_msg.twist.linear.x = 0.5 * cos(yaw);  
        cmd_vel_msg.twist.linear.y = 0.5 * sin(yaw);  
        cmd_vel_msg.twist.linear.z = 0.0;  
        cmd_vel_msg.twist.angular.x = 0.0; 
        cmd_vel_msg.twist.angular.y = 0.0; 
        cmd_vel_msg.twist.angular.z = 0.01; 
        cmd_vel_pub.publish(cmd_vel_msg);
        ros::spinOnce();
          }
            }
            else{
                ROS_INFO("Unknown Case going straight");

          geometry_msgs::TwistStamped cmd_vel_msg;
        cmd_vel_msg.header.stamp = ros::Time::now();
        cmd_vel_msg.twist.linear.x = 0.5 * cos(yaw);  
        cmd_vel_msg.twist.linear.y = 0.5 * sin(yaw);  
        cmd_vel_msg.twist.linear.z = 0.0;  
        cmd_vel_msg.twist.angular.x = 0.0; 
        cmd_vel_msg.twist.angular.y = 0.0; 
        cmd_vel_msg.twist.angular.z = 0.0; 
        cmd_vel_pub.publish(cmd_vel_msg);
        ros::spinOnce();


            }

        }

        

    

        }

      }
      std::cout<<angle;
      ros::spinOnce();
        

        


    



		rate.sleep();

            ros::spinOnce();
	
	return 0;
}