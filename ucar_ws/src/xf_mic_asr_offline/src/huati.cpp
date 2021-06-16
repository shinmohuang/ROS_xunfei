#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc,char **argv)
{
 ros::init(argc,argv,"huati");
 ros::NodeHandle nh;
 ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
 ros::Rate loop_rate(10);
 int count=0;
 while(ros::ok())
 {
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;   
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
 
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.1;
  
  //ROS_INFO("shuchu11");
  pub.publish(twist);
  ros::spinOnce();
  loop_rate.sleep();
  ++count;
 }
 return 0;
}

