//RobotCraft - Example to control 1 RGB Led on ROS.

#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"

int main (int argc, char **argv){
    
    ros::init(argc, argv, "led_test_node");
    ros::NodeHandle n;
    ros::Publisher led_pub = n.advertise<std_msgs::UInt8MultiArray>("rgb_topic", 5);
    ros::Rate loop_rate(5);
    
    while (ros::ok()){
        
            std_msgs::UInt8MultiArray array_msg;
            
            //Clear array_msg:
            array_msg.data.clear();
            
            //push data to the array_msg.data:
            array_msg.data.push_back(rand()%255); //red
            array_msg.data.push_back(rand()%255); //green
            array_msg.data.push_back(rand()%255); //blue
            
            led_pub.publish(array_msg);
            
            ROS_INFO("Publishing new LED values.");
            
            ros::spinOnce();
            loop_rate.sleep();
    }
    
    return 0;
}