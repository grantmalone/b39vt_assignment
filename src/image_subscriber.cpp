#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "b39vt_assignment/image_processing.hpp"

class ImageSubscriber
{
	// By default, variables and methods are private
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  
  cv_bridge::CvImagePtr cv_ptr;
  
  const std::string OPENCV_WINDOW;
  
public:
	// Public variables - it is a good practice to separate the declaration of
	// variables from the declaration/definition of methods
	bool data_valid;
  
public:
	// Public methods
  ImageSubscriber() : it_(nh_), data_valid(false), OPENCV_WINDOW("Image window")
  {
    // Subscribe to input video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &ImageSubscriber::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageSubscriber()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      data_valid = true;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // Update GUI Window - uncomment if you want to visualize the subscribed
    // image...
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);
  }
  
  cv::Mat getImage(){
  	return cv_ptr->image;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_subscriber");
  ImageSubscriber ic;
  
  std::string image_array [8] = {"/home/turtlebot/green_helmet.png" , 
  															 "/home/turtlebot/biohazard.png" ,
  															 "/home/turtlebot/danger.png" ,
  															 "/home/turtlebot/fire.png" ,
  															 "/home/turtlebot/red_helmet.png" ,
  															 "/home/turtlebot/radioactive.png" ,
  															 "/home/turtlebot/smoking.png" ,
  															 "/home/turtlebot/toxic.png"};
  
  for( int i = 0; i < 8; i++){
  cv::Mat sign = cv::imread(image_array[i]);
  
  while (ros::ok())
  {
		if (ic.data_valid)
		{
			cv::resize(sign, sign, cv::Size(50,50));
			cv::Mat im = ic.getImage();
			templateMatching(im, sign);
			
		}
  	ros::spinOnce();
  }
  
  }
  return 0;
}