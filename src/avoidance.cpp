#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/Led.h"

#include "b39vt_assignment/avoid.hpp"

//subscribes to laser
LaserSubscriber::LaserSubscriber(ros::NodeHandle n)
    : n_(n)
    , data_valid(false)
    , laser_sub_(n_, "base_scan", 10)
    , laser_notifier_(laser_sub_, listener_, "base_link", 10)
{
    sub_ = n_.subscribe("scan", 1000, &LaserSubscriber::laserCb, this);
    vel_pub_ = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); //publishes velocity commands to base
    led_pub1 = n.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1000);
    led_pub2 = n.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1000);
}

LaserSubscriber::~LaserSubscriber() {}

void LaserSubscriber::laserCb(const sensor_msgs::LaserScan& msg)
{
    try {
        publish(msg);
    }
    catch (tf::TransformException& e) {
        std::cout << e.what();
        return;
    }
}

void LaserSubscriber::publish(const sensor_msgs::LaserScan& msg)
{
		
    geometry_msgs::Twist vel;
		
		//creates initial velocity (no obstacles)
    float linearx = 0;
    vel.linear.x = linearx;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

		//looks at the central 60 degree (30 each of centre)
    float min = 10000;
    float minr = 10000;
    float minl = 10000;
        
    for (int i = 170.33; i < (msg.ranges.size() - 170.33); i++) {
        if (msg.ranges[i] < min && msg.ranges[i] > 0.15) {
            min = msg.ranges[i];
        }
        
    }
        //looks at right side (60 degrees) - turns left if obstacle on right
        for (int i = 340.67; i < (msg.ranges.size()); i++) {
            if (msg.ranges[i] < minr && msg.ranges[i] > 0.15) {
                minr = msg.ranges[i];
            }
            
        }
        //looks at left side (60 degree) - turns right if obstacle no right
        for (int i = 0; i < (msg.ranges.size() - 340.67); i++) {
            if (msg.ranges[i] < minl && msg.ranges[i] > 0.15) {
                minl = msg.ranges[i];    
            }
        }
        
        if (min > 0.30){
        vel.linear.x = 0.2;
        }
        
        else if (min < 0.3 and minl< 0.3 and minr<0.3){
        ROS_INFO_STREAM("in corner if");
         ros::Time start2 = ros::Time::now();
            while (ros::Time::now() - start2 < ros::Duration(0.1)) { //reverses for 0.3 time increments
                vel.linear.x = -0.1 ; //sets reverse speed
								vel_pub_.publish(vel);
            }
						vel.linear.x = 0;
						ros::Time turn2 = ros::Time::now();
            while (ros::Time::now() - turn2 < ros::Duration(0.3)) { //sets turn time to 0.2 time increments
            
						//once reversed, rechecks left and right hand side to determine which way to progress
            if (minl > minr) {
            //0.9 originally
                vel.angular.z = -0.8;
            }
            else {
                vel.angular.z = 0.8;
            }
            vel_pub_.publish(vel);	
          }
          vel.angular.z = 0;
         
        }
        
        else if (min < 0.30){
        		ROS_INFO_STREAM("min; " << min);
        		ros::Time turn = ros::Time::now();
            while (ros::Time::now() - turn < ros::Duration(0.05)) { //sets turn time to 0.2 time increments
            
						//once reversed, rechecks left and right hand side to determine which way to progress
            if (minl > minr) {
            //0.9 originally
                vel.angular.z = -0.8;
            }
            else {
                vel.angular.z = 0.8;
           		 }
            	
           }
        }
        
        	vel_pub_.publish(vel);
 }



int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoid");
    ros::NodeHandle n;

    LaserSubscriber ls(n);

    ros::spin();

    return 0;
}
