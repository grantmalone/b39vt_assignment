#ifndef IMAGE_PROCESSING_B39VT_ASSIGNMENT_HPP
#define IMAGE_PROCESSING_B39VT_ASSIGNMENT_HPP

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <ros/console.h>
#include "geometry_msgs/PointStamped.h"

std::vector<cv::DMatch> templateMatching(const cv::Mat& im, const cv::Mat& templ);
// Your code goes here

# endif // IMAGE_PROCESSING_B39VT_ASSIGNMENT_HPP
