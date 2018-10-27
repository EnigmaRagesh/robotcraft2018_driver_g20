#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"

float x_r,y_r,theta_r,f_d,l_d,r_d;
double x_in = 0, y_in = 0, theta_in = 0;
double vx = 0.1;
double vy = 0.0;
double vth = 0.0;
int Led1_R = 0, Led1_G = 150, Led1_B = 100;
int Led2_R = 0, Led2_G = 150, Led2_B = 100;

void poseCallback(const geometry_msgs::Pose2D& Pose2D_msgs)
{
	x_r = Pose2D_msgs.x;
	y_r = Pose2D_msgs.y;
	theta_r = Pose2D_msgs.theta;
}

void front_distance_callback(const std_msgs::Float32& front_distance_msgs)
{
	f_d = front_distance_msgs.data;
}

void left_distance_callback(const std_msgs::Float32& left_distance_msgs)
{
	l_d = left_distance_msgs.data;
}

void right_distance_callback(const std_msgs::Float32& right_distance_msgs)
{
	r_d = right_distance_msgs.data;
}

int main(int argc, char **argv)
{
    
  	ros::init(argc, argv, "robot_driver");
  	ros::NodeHandle n;
	tf::TransformBroadcaster odom_broadcaster;
	//Subscribers
	ros::Subscriber pose_sub = n.subscribe("/pose", 10, poseCallback);
	ros::Subscriber front_distance_sub = n.subscribe("/front_distance", 10,front_distance_callback);
	ros::Subscriber right_distance_sub = n.subscribe("/right_distance", 10,right_distance_callback);
	ros::Subscriber left_distance_sub = n.subscribe("/left_distance", 10,left_distance_callback);
	
	//Publishers
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom",10 );
	ros::Publisher ir_front_pub = n.advertise<sensor_msgs::Range>("/ir_front_sensor",10);
	ros::Publisher ir_left_pub  = n.advertise<sensor_msgs::Range>("/ir_left_sensor",10);
	ros::Publisher ir_right_pub = n.advertise<sensor_msgs::Range>("/ir_right_sensor",10);
	ros::Publisher rgb_leds_pub = n.advertise<std_msgs::UInt8MultiArray>("/rgb_leds",60);
	ros::Publisher initial_pose_pub = n.advertise<geometry_msgs::Pose2D>("/initial_pose",10);
	
	//Message initialization
	nav_msgs::Odometry odom_msg;
	sensor_msgs::Range ir_front_msg, ir_left_msg, ir_right_msg;
    std_msgs::UInt8MultiArray rgb_leds_msg;
	geometry_msgs::Pose2D initial_pose_msg;	
	ros::Time current_time, last_time;
  	current_time = ros::Time::now();
  	last_time = ros::Time::now();
	ros::Rate r(10);

	while (ros::ok())
	{
		current_time = ros::Time::now();	
		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(theta_r) - vy * sin(theta_r)) * dt;
    	double delta_y = (vx * sin(theta_r) + vy * cos(theta_r)) * dt;
    	double delta_th = vth * dt;

    	x_r+= delta_x;
    	y_r+= delta_y;
    	theta_r+= delta_th;		

		//Publish the transform over tf
	    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_r);    
	    geometry_msgs::TransformStamped odom_trans;
	    odom_trans.header.stamp = current_time;
	    odom_trans.header.frame_id = "odom";
	    odom_trans.child_frame_id = "base_link";
  	odom_trans.transform.translation.x = x_r/100;
    	odom_trans.transform.translation.y = y_r/100;
    	odom_trans.transform.translation.z = 0.0;
    	odom_trans.transform.rotation = odom_quat;
		//sendig the transform
		odom_broadcaster.sendTransform(odom_trans);

		//set the postion	
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = "odom";
		odom_msg.pose.pose.position.x = x_r;
		odom_msg.pose.pose.position.y = y_r;
 		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = odom_quat;

		 //set the velocity
	    odom_msg.child_frame_id = "base_link";
	    odom_msg.twist.twist.linear.x = vx;
	    odom_msg.twist.twist.linear.y = vy;
	    odom_msg.twist.twist.angular.z = vth;

	    //publish the odometry message
	    odom_pub.publish(odom_msg);
		
		//publish front distance sensor
		ir_front_msg.header.stamp = ros::Time::now();
		ir_front_msg.header.frame_id = "front_ir";
		ir_front_msg.radiation_type = 1,                    
		ir_front_msg.field_of_view = 0.034906585;
		ir_front_msg.min_range = 0.1;
		ir_front_msg.max_range = 0.8;
		ir_front_msg.range = f_d;
		ir_front_pub.publish(ir_front_msg);	
		
		//publish left distance sensor
		ir_left_msg.header.stamp = ros::Time::now();
		ir_left_msg.header.frame_id = "left_ir";
		ir_left_msg.radiation_type = 1,                    
		ir_left_msg.field_of_view = 0.034906585;
		ir_left_msg.min_range = 0.1;
		ir_left_msg.max_range = 0.8;
		ir_left_msg.range = l_d;
		ir_left_pub.publish(ir_left_msg);
		
		//publish right distance sensor
		ir_right_msg.header.stamp = ros::Time::now();
		ir_right_msg.header.frame_id = "right_ir";
		ir_right_msg.radiation_type = 1,                    
		ir_right_msg.field_of_view = 0.034906585;
		ir_right_msg.min_range = 0.1;
		ir_right_msg.max_range = 0.8;
		ir_right_msg.range = r_d;
		ir_right_pub.publish(ir_right_msg);				

		//publish rgb led data
		rgb_leds_msg.data.clear();
		rgb_leds_msg.data.push_back(Led1_R);
		rgb_leds_msg.data.push_back(Led1_G);
		rgb_leds_msg.data.push_back(Led1_B);
		rgb_leds_msg.data.push_back(Led2_R);
		rgb_leds_msg.data.push_back(Led2_G);
		rgb_leds_msg.data.push_back(Led2_B);

		rgb_leds_pub.publish(rgb_leds_msg);	

		//publish initial pose data
		initial_pose_msg.x = x_in;
		initial_pose_msg.y = y_in;
		initial_pose_msg.theta = theta_in;		
		initial_pose_pub.publish(initial_pose_msg);
			
   		last_time = current_time;
    	ros::spinOnce();
		r.sleep();
	}  
	return 0;
}
