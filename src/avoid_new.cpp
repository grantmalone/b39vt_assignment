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
    vel_pub_ = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 3); //publishes velocity commands to base
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
    float linearx = 0.2;
    vel.linear.x = linearx;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

		//looks at the central 60 degree (30 each of centre)
    float min = 10000;
    for (int i = 170.33; i < (msg.ranges.size() - 170.33); i++) {
        if (msg.ranges[i] < min && msg.ranges[i] > 0.15) {
            min = msg.ranges[i];
            
             	 kobuki_msgs::Led msg1;
               kobuki_msgs::Led msg2;
               msg1.value = 1;
               msg2.value = 1;
               led_pub1.publish(msg1);
               led_pub2.publish(msg2);
        }
    }

    vel_pub_.publish(vel);
    if (min < 0.3 && min > 0.10) {
        vel.linear.x = 0;
        vel_pub_.publish(vel);

        float minr = 10000;
        float minl = 10000;
        
        //looks at right side (60 degrees) - turns left if obstacle on right
        for (int i = 340.67; i < (msg.ranges.size()); i++) {
            if (msg.ranges[i] < minr && msg.ranges[i] > 0.15) {
                minr = msg.ranges[i];
                
               //left indicator
               kobuki_msgs::Led msg1;
               kobuki_msgs::Led msg2;
               msg1.value = 2;
               msg2.value = 0;
               led_pub1.publish(msg1);
               led_pub2.publish(msg2);
               
            }
        }
        
        //looks at left side (60 degree) - turns right if obstacle no right
        for (int i = 0; i < (msg.ranges.size() - 340.67); i++) {
            if (msg.ranges[i] < minl && msg.ranges[i] > 0.15) {
                minl = msg.ranges[i];
                
               //right indicator
               kobuki_msgs::Led msg1;
               kobuki_msgs::Led msg2;
               msg1.value = 0;
               msg2.value = 2;
               led_pub1.publish(msg1);
               led_pub2.publish(msg2);
            }
        }
        float angularv = 0;

        //incase obstacales are infront and both sides - reverses, then rescans area
        //0.25 & 0.5 original
        if (minr < 0.3 && minl < 0.3 && min < 0.3) {

            ros::Time start2 = ros::Time::now();
            while (ros::Time::now() - start2 < ros::Duration(0.3)) { //reverses for 0.3 time increments
                vel.linear.x = linearx / -2; //sets reverse speed
								vel_pub_.publish(vel); 
            }

						ros::Time turn2 = ros::Time::now();
            while (ros::Time::now() - turn2 < ros::Duration(0.2)) { //sets turn time to 0.2 time increments
            
						//once reversed, rechecks left and right hand side to determine which way to progress
            if (minl > minr) {
            //0.9 originally
                angularv = -0.9;
            }
            else {
                angularv = 0.9;
            }
            	vel.angular.z = angularv;
            	vel_pub_.publish(vel);
            	}
        }

        if (minl > minr) {
            angularv = -0.9;
        }
        else {
            angularv = 0.9;
        }

        vel.angular.z = angularv;
        vel_pub_.publish(vel);
        vel.angular.z = 0;
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoid");
    ros::NodeHandle n;

    LaserSubscriber ls(n);

    ros::spin();

    return 0;
}
