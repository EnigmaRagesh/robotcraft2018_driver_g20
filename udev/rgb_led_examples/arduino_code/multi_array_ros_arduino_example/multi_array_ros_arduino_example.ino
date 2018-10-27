/*
 * multi_array_ros_arduino_example
 * Subscribe std_msgs::UInt8MultiArray
 * 
 */

#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>

ros::NodeHandle  nh;


void messageCb( const std_msgs::UInt8MultiArray& array_msg){

    /* Control RGB Led, with values between 0 - 255 */
    // array_msg.data[]={r1,g1,b1,r2,g2,b2}

    // Just for debug 1 index of the array (convert to string) 
    char final_array[20];
    String str1 = String (array_msg.data[0]);
    str1.toCharArray(final_array, 5);
    nh.loginfo(final_array);

}

ros::Subscriber<std_msgs::UInt8MultiArray> sub_rgb_leds_info("/topic_leds", messageCb );

void setup()
{
  nh.initNode();
  nh.subscribe(sub_rgb_leds_info);
}

void loop()
{

   nh.spinOnce();   // update available publish msg to ros pc side

}
