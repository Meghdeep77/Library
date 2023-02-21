#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "math.h"
#include <bits/stdc++.h>

using namespace std;

double ow;
double ox;
double oy;
double oz;
double siny_cosp;
double cosy_cosp;
double yaw;
double yawd = yaw * 180 / 3.1415;




void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{



   ow = msg->orientation.w ;
   ox = msg->orientation.x;
   oy = msg->orientation.y;
   oz = msg->orientation.z;
   siny_cosp = 2 * (ow * oz + ox * oy);
   cosy_cosp = 1 - 2 * (oy * oy + oz * oz);
   yaw = std::atan2(siny_cosp, cosy_cosp);

   yawd = yaw * 180 / 3.1415;


  ROS_INFO("Orientation: (%f,%f, %f, %f) yaw = %f", ox, oy, oz,ow, yawd);

}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  double lat = msg->latitude * 10000;
  double lon = msg->longitude * 10000;
  double alt = msg->altitude; 
  //double oz = msg->orientation.z;

  ROS_INFO("Position: (%f,%f, %f)", lat, lon, alt);
}


 int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;

    ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //ros::Publisher imuPub = n.advertise<geometry_msgs::Vector3>("/Imu_lin_pub", 1000);
    ros::Subscriber sub1 = n.subscribe("/imu", 1000, imuCallback);
    ros::Subscriber gpssub = n.subscribe("/gps/fix", 1000, gpsCallback);
    ros::Rate loop_rate(20);

    geometry_msgs::Twist msg;
  
 

 
    while (ros::ok())
    {   
        
        msg.angular.z = 0.1;
        velPub.publish(msg);
        if(abs(yawd - 100) < 3){
          msg.angular.z = 0;
          velPub.publish(msg);

        }

        
        //imuPub.publish(pub_data);
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}



