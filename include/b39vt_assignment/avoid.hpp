#ifndef AVOID_B39VT_ASSIGNMENT_HPP
#define AVOID_B39VT_ASSIGNMENT_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/Led.h"

class LaserSubscriber
{
	// By default, variables and methods are private
	ros::Subscriber sub_;
 	ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher led_pub1;
  ros::Publisher led_pub2;
 
public:
	// Public variables - it is a good practice to separate the declaration of
	// variables from the declaration/definition of methods
	bool data_valid;
  
public:
	// Public methods
  LaserSubscriber(ros::NodeHandle n);
  ~LaserSubscriber();
  
  void laserCb(const sensor_msgs::LaserScan& msg);
  void publish(const sensor_msgs::LaserScan& msg);
};
// Your code goes here

# endif // AVOID_B39VT_ASSIGNMENT_HPP
