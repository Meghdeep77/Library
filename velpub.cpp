#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "math.h"
#include <bits/stdc++.h>

class Planner {
public:
    Planner(ros::NodeHandle *nh);

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

private:
    ros::NodeHandle* nh;
    ros::Subscriber sub1;
    double ow;
    double ox;
    double oy;
    double oz;
    double siny_cosp;
    double cosy_cosp;
    double yaw;
    double yawd = yaw * 180 / 3.1415;
    double flag;
    double so;
};

Planner::Planner(ros::NodeHandle *nh) {
    this->nh = nh;
    sub1 = nh->subscribe("/imu", 1000, &Planner::imuCallback, this);
}

void Planner::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ow = msg->orientation.w;
    ox = msg->orientation.x;
    oy = msg->orientation.y;
    oz = msg->orientation.z;
    siny_cosp = 2 * (ow * oz + ox * oy);
    cosy_cosp = 1 - 2 * (oy * oy + oz * oz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    yawd = yaw * 180 / M_1_PI;

    flag++;

    if (flag == 1) {
        so = yawd;
    }

    ROS_INFO("Orientation: (%f,%f, %f, %f) yaw = %f initial yaw = %f", ox, oy, oz, ow, yawd, so);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    Planner rm(&nh);
    
    ros::spin();

    return 0;
}
