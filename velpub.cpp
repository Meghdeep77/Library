#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/LaserScan.h"
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
bool object_ahead = false;
bool object_left = false;
bool object_right = false;
bool object_back_left = false;
bool wall_following = false;






class Planner {
public:
    Planner(ros::NodeHandle *nh);

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    
private:
    ros::NodeHandle* nh;
    ros::Subscriber sub1;
    ros::Subscriber gpssub;
    ros::Subscriber lasersub;
    
    };

Planner::Planner(ros::NodeHandle *nh) {
    this->nh = nh;
    sub1 = nh->subscribe("/imu", 1000, &Planner::imuCallback, this);
    gpssub = nh->subscribe("/gps/fix", 1000, &Planner::gpsCallback, this);
    lasersub = nh->subscribe("/scan", 1000, &Planner::laserCallback, this);

}

void Planner::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ow = msg->orientation.w;
    ox = msg->orientation.x;
    oy = msg->orientation.y;
    oz = msg->orientation.z;
    siny_cosp = 2 * (ow * oz + ox * oy);
    cosy_cosp = 1 - 2 * (oy * oy + oz * oz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    yawd = yaw * 180 / 3.1415;

    flag++;

    if (flag == 1) {
        so = yawd;
    }

    ROS_INFO("Orientation: (%f,%f, %f, %f) yaw = %f initial yaw = %f", ox, oy, oz, ow, yawd, so);
}
void Planner::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
   lat = msg->latitude;
   lon = msg->longitude;
   double alt = msg->altitude;
   
  //double oz = msg->orientation.z;
  dist = sqrt(pow(lat - y, 2) + pow(lon - x, 2));
  
  if(flag2 == 0){
    start_lon = msg->longitude;
    start_lat = msg->latitude;
    flag2 ++;
    }
  if(flag2 == 10){
    start_lon = lon;
    start_lat = lat;
    flag2++;

  }
  sdist  = sqrt(pow(start_lat - lat, 2) + pow(start_lon - lon, 2)); 
ROS_INFO("Position: (%f,%f, %f) distance = %f .starting position(%f,%f), sdist = %f", lon, lat, alt, dist,start_lon,start_lat,sdist);
  
}

void Planner::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  size = msg -> ranges.size();
    for(int i = 276; i< 390;i++){

            obstacle_angle = i/2;
            fdist = msg-> ranges[i];
            if(fdist<30){
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(fdist<1.5){
            object_ahead = true;
            wall_following = true;
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);

            break;
            }
            else{
            object_ahead = false;

           }
           
           
               }
               else{
            object_ahead = false;}
               
               }
     for(int i = 390; i< 480;i++){

            obstacle_angle = i/2;
            lb_dist = msg-> ranges[i];
            if(lb_dist<30){
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(lb_dist<1.5){
            object_left = true;
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);

            break;
            }
            else{
            object_left = false;

           }
           
           
               }
            else{
            object_left = false;

           }
               
               }

    for(int i = 480; i< 600;i++){

            obstacle_angle = i/2;
            l_dist = msg-> ranges[i];
            if(l_dist<30){
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(l_dist<1.5){
            object_back_left = true;
            ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);

            break;
            }
            else{
            object_back_left= false;

           }
           }
           else{
            object_back_left= false;

           }
           
               }

    
    
    
  
    
    ROS_INFO("size = %f",size);
  
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    Planner rm(&nh);
    ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Rate loop_rate(20);
    geometry_msgs::Twist msg;
    if(flag3 == 0){
    cout<<"Enter Longitude";
    cin>>x;
    cout<<"Enter Lattitude";
    cin>>y;
    ros::spinOnce();
    flag3++;
    }
    
    
    
    while (ros::ok()){
    


     if(object_ahead == false && object_left == false && object_back_left == false ){
        wall_following = false;
        flag2 = 10;
        ros::spinOnce();

    }


    if(wall_following == true){
      ROS_INFO("wall following = %d",wall_following);
      ROS_INFO("Avoiding obstacle");
      ROS_INFO("object ahead= %d, object left = %d, object back left = %d",object_ahead,object_left,object_back_left);

        if(wall_following == true && object_ahead == true){
            msg.linear.x = 0;
            msg.angular.z = -0.1;
            ROS_INFO("Obstacle ahead");
            velPub.publish(msg);
        }
        else if(wall_following == true && object_left == true && object_ahead == false){
            msg.linear.x = 0.5;
            msg.angular.z = 0;
            ROS_INFO("Obstacle on left");
            velPub.publish(msg);
        }
        else if(wall_following == true && object_left == false && object_ahead == false && object_back_left == true){
            msg.linear.x = 0;
            msg.angular.z = 0.1;
            ROS_INFO("Corner");
            velPub.publish(msg);
        }
    


        
        ros::spinOnce();
        loop_rate.sleep();


    }
    else{
      ROS_INFO("Going to Point");
      angle = atan2(y-lat,x -lon)* 180 / 3.1415;
      cout<<angle;
      ros::spinOnce();
        if(angle > so && dist > 0.000010 && sdist < 0.000010){
        msg.angular.z = 0.1;
        velPub.publish(msg);
        }
        if(sdist >= 0.000010){
        msg.angular.z = 0;
        velPub.publish(msg);
        }

        if (angle < so && sdist < 0.000010 && dist > 0.000010){
        msg.angular.z = -0.1;
        velPub.publish(msg);
        }
        if(abs(yawd - angle) < 1 && dist > 0.000010){
          msg.angular.z = 0;
          msg.linear.x = 1;
          velPub.publish(msg);

        }
        if(abs(yawd - angle) > 0.5 && sdist > 0.000020){

          if(yawd > angle ){

          msg.angular.z = -0.2;
          msg.linear.x = 1;
          velPub.publish(msg);
          }

          if(yawd < angle ){

          msg.angular.z = 0.2;
          msg.linear.x = 1;
          velPub.publish(msg);
          }
        }

       if(dist <= 0.000010){
          msg.linear.x = 0 ;
          msg.angular.z = 0;
          velPub.publish(msg);

        }
       ros::spinOnce();
        loop_rate.sleep();
    }
                        }
      return 0;

      }