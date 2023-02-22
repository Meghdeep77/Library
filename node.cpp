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
double lat;
double lon;
double x ;
double y ;
double dist;
double angle;





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
   lat = msg->latitude;
   lon = msg->longitude;
   double alt = msg->altitude; 
  //double oz = msg->orientation.z;
  dist = sqrt(pow(lat - y, 2) + pow(lon - x, 2));
  
  ROS_INFO("Position: (%f,%f, %f) distance = %f", lon, lat, alt, dist);
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
    cout<<"Enter Longitude";
    cin>>x;
    cout<<"Enter Lattitude";
    cin>>y;
    angle = atan2(y,x)* 180 / 3.1415;
    cout<<angle;


  

    
  
 

 
    while (ros::ok())
    {   
        
        msg.angular.z = 0.1;
        velPub.publish(msg);
        if(abs(yawd - angle) < 1 && dist > 0.000010){
          msg.angular.z = 0;
          msg.linear.x = 1 ;
          velPub.publish(msg);

        }

        if(dist <= 0.000010){
          msg.linear.x = 0 ;
          msg.angular.z = 0;
          velPub.publish(msg);

        }


        
        //imuPub.publish(pub_data);
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}



