#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

float x_r,y_r, theta_r, vx, vy, vth;
ros::Time current_time, last_time;
nav_msgs::Odometry odom_msg;

void go_straight()
{	
 	odom_msg.twist.twist.linear.x =2.0;
	odom_msg.twist.twist.angular.z=0.0;
	
}
void turn()
{
	odom_msg.twist.twist.linear.x =0.0;
 	odom_msg.twist.twist.angular.z=1.57;
}
void stop()
{

	odom_msg.twist.twist.linear.x =0.0;
 	odom_msg.twist.twist.angular.z=0.0;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "square_test");
	ros::NodeHandle n;
  	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom",10 );
 	while (ros::ok())
  	{
	go_straight();
	odom_pub.publish(odom_msg);
	ros::Duration(5.0);
	turn();
	odom_pub.publish(odom_msg);
	ros::Duration(5.0);
	stop();
	odom_pub.publish(odom_msg);
	ros::Duration(5.0);
	}
    ros::spinOnce();
	return 0;
  }
