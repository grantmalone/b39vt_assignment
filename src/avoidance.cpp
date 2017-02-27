#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Twist.h"

#include "b39vt_assignment/avoid.hpp"

LaserSubscriber::LaserSubscriber(ros::NodeHandle n) : n_(n), 
	data_valid(false), laser_sub_(n_, "base_scan", 10),
	laser_notifier_(laser_sub_,listener_, "base_link", 10){
  	sub_ = n_.subscribe("scan", 1000, &LaserSubscriber::laserCb, this);
  	scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
  	vel_pub_ = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 3); 
}

LaserSubscriber::~LaserSubscriber() {  }
  

void LaserSubscriber::laserCb(const sensor_msgs::LaserScan& msg)
{
    sensor_msgs::PointCloud cloud;
    
    ROS_INFO("In callback function");
    try
    {
        ROS_INFO("Within try catch");
        //projector_.transformLaserScanToPointCloud("base_link", msg, cloud, listener_);
        publish(msg);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
}
   
void LaserSubscriber::publish(const sensor_msgs::LaserScan& msg){ 
   
	geometry_msgs::Twist vel;
   
  float linearx = 0.25;
	vel.linear.x = linearx;
	vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = 0;
	
	ROS_INFO("Publishing message");

  float min = 10000;
  float max = 0;
  for (int i = 127.75;i<(msg.ranges.size()-127.75);i++){
  if(msg.ranges[i] < min && msg.ranges[i]>0.15){min = msg.ranges[i];}
  if(msg.ranges[i] > max){max = msg.ranges[i];}
  }
  
  
  ROS_WARN_STREAM("Min value :"<< min);
  ROS_WARN_STREAM("Max value :"<< max);
  
  vel_pub_.publish(vel);
  if( min< 0.5 && min > 0.15){
  vel.linear.x = 0;
  vel_pub_.publish(vel);
  
  float minr = 10000;
  float minl = 10000;
  //right side
  for (int i = 383.25;i<(msg.ranges.size());i++){
  if(msg.ranges[i] < minr && msg.ranges[i]>0.15){minr = msg.ranges[i];}
  }
  //left side
  for (int i = 0;i<(msg.ranges.size()-383.25);i++){
  if(msg.ranges[i] < minl && msg.ranges[i]>0.15){minl = msg.ranges[i];}
  }
  float angularv = 0;
  
  //both
  if(minr<0.5 && minl<0.5){
  
  ros::Time start2 = ros::Time::now();
	while(ros::Time::now() - start2 < ros::Duration(0.2))
	{
	vel.linear.x = linearx/-2;
	vel_pub_.publish(vel);
	}
  if(minl > minr){angularv = -0.9;}
  else {angularv = 0.9;}
  } 
  
  if(minl > minr){angularv = -0.9;}
  else {angularv = 0.9;}
  
  
  vel.angular.z = angularv;
  vel_pub_.publish(vel);
  
  };
  
  
}
  

int main(int argc, char** argv)
{
	ros::init(argc, argv, "avoid");
  ros::NodeHandle n;
  
  LaserSubscriber ls(n);
	//LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}
