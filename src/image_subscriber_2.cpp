#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/PointStamped.h"
#include <ros/console.h>
#include "b39vt_assignment/image_processing.hpp"

class ImageSubscriber {
    // By default, variables and methods are private
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;

    cv_bridge::CvImagePtr cv_ptr; //pointer for
    cv_bridge::CvImagePtr cv_ptr_; //pointer depth
    const std::string OPENCV_WINDOW;

public:
    // Public variables - it is a good practice to separate the declaration of
    // variables from the declaration/definition of methods
    bool data_valid;
    bool depth_valid;
    

public:
    // Public methods
    ImageSubscriber()
        : it_(nh_)
        , data_valid(false)
    {
        // Subscribe to input video feed
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
            &ImageSubscriber::imageCb, this);
            
            
        depth_sub_ = it_.subscribe("/camera/depth_registered/image", 1,
            &ImageSubscriber::depthCb, this);
            
    }

    ~ImageSubscriber()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }


    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //assigns pointer
            data_valid = true;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    
     void depthCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_64FC1); //assigns pointer
            depth_valid = true;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    
    
    //retrieves webcam image
    cv::Mat getImage()
    {
        return cv_ptr->image;
    }
    
    cv::Mat getDepth()
    {
    	return cv_ptr_->image;
    }
};

int main(int argc, char** argv)
{
    try {
    	 ROS_INFO_STREAM("debug " );
        ros::init(argc, argv, "image_subscriber");
        ros::NodeHandle n;
        ImageSubscriber ic;
        ros::Publisher depth_pub_ = n.advertise<geometry_msgs::PointStamped>("depth", 3);

        std::string image_array[8] = { "/home/turtlebot/green_helmet.png",
            "/home/turtlebot/biohazard.png",
            "/home/turtlebot/danger.png",
            "/home/turtlebot/smoking.png",
            "/home/turtlebot/red_helmet.png",
            "/home/turtlebot/radioactive.png",
            "/home/turtlebot/toxic.png",
            "/home/turtlebot/fire.png" };

        while (ros::ok()) {
            if (ic.data_valid and ic.depth_valid) {
                cv::Mat im = ic.getImage();
                cv::Mat dep = ic.getDepth();
                
                
               /* dep.height
                dep.width
                
                dep.rows
                dep.cols*/
                
                int maximum = 0;
                int winner = 99;
                int match_signs[8];
                for (int i = 0; i < 8; i++) {
                    cv::Mat sign = cv::imread(image_array[i]);
                    cv::resize(sign, sign, cv::Size(450, 450));
                    std::vector<cv::DMatch> good_matches = templateMatching(im, sign);
                    match_signs[i] = good_matches.size();

                    if (good_matches.size() > maximum) {
                        maximum = good_matches.size();
                        winner = i;
                    }
                }
                std::string win;
                
                if (winner == 0) {
                    win = "Green Helmet";
                }
                else if (winner == 1) {
                    win = "Biohazard";
                }
                else if (winner == 2) {
                    win = "Danger";
                }
                else if (winner == 3) {
                    win = "Smoking";
                }
                else if (winner == 4) {
                    win = "Red Helmet";
                }
                else if (winner == 5) {
                    win = "Radioactive";
                }
                else if (winner == 6) {
                    win = "Toxic";
                }
                else if (winner == 7) {
                    win = "Fire";
                }
                else {
                    ROS_INFO("no matches found");
                }
                if ((match_signs[winner] < 7) or (dep.at<double>(160,240) > 0.8)) {
                    ROS_INFO("no matches found  ");
                }
                else {
                    ROS_INFO("best match: %s\n", win.c_str());
                    ROS_INFO_STREAM("NO OF MATCHES: " << match_signs[winner]);
                    if (std::isnan(dep.at<double>(320,240))){
                    dep = 0.50;
                    ROS_INFO_STREAM("DEPTH:::: " << dep.at<double>(320,240));
                    } 
                    else{
                    ROS_INFO_STREAM("DEPTH: " << dep.at<double>(320,240));
                     geometry_msgs::PointStamped point;
                    
                    point.point.x = 0;
                    point.point.y = 0;
                    point.point.z = dep.at<double>(320,240);
                    
                    point.header.frame_id = "camera_rgb_optical_frame";
                    depth_pub_.publish(point);
                    }
                    
                   
                }
            }
            ros::spinOnce();
        }
    }
    catch (cv::Exception& e) {
         ROS_INFO("no matches found");
    }
    return 0;
}
