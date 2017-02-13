#include "b39vt_assignment/image_processing.hpp"

std::vector<cv::DMatch> templateMatching(const cv::Mat& im, const cv::Mat& sign)
{
	int minHessian = 400;
  //int method = 0;
  //cv::ORB orb;
  cv::Mat descriptors_1, descriptors_2;
  cv::OrbDescriptorExtractor extractor;
 
  
  cv::OrbFeatureDetector detector;//(50, 1.2f, 8, 71, 0, 2, 0, 81);

  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    
  detector.detect( sign, keypoints_1 );
  detector.detect( im, keypoints_2 );
	
	//if( method == 0 ) {
    detector.detect(sign, keypoints_1);
    extractor.compute(sign, keypoints_1, descriptors_1);
 // }
	
	int result_cols = im.cols - sign.cols + 1;
	int result_rows = im.rows - sign.rows + 1;

	cv::Mat result;
	cv::Mat descriptors_object;
	result.create( result_cols, result_rows, CV_32FC1 );

  if( !sign.data || !im.data )
   {  std::cout<<"no images loaded"; }
   
  //-- Step 2: Calculate descriptors (feature vectors)
  extractor.compute( sign, keypoints_1, descriptors_1 );
  extractor.compute( im, keypoints_2, descriptors_2 );

  //-- Step 3: Matching descriptor vectors with a brute force matcher
  std::vector<cv::DMatch> matches;
  cv::BFMatcher matcher(cv::NORM_HAMMING2, true);
  //std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );
  
   
  
    //-- Quick calculation of max and min distance between keypoints
      
  double max_dist = 1000; double min_dist = 100;
    
  for( int i = 0; i < descriptors_object.rows; i++)
  { double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
  }

	std::vector< cv::DMatch >good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  { 
  	if( matches[i].distance < (max_dist/1) )
    { 
    	good_matches.push_back( matches[i]); 
    }
  }


  //-- Draw matches
  cv::Mat img_matches;
  cv::drawMatches( sign, keypoints_1, im, keypoints_2, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                std::vector<char>(), cv::DrawMatchesFlags::DEFAULT  /*, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS*/ );

		int i = matches.size();
		std::stringstream ss;
    ss << i;
  

cv::putText(img_matches,ss.str(), cv::Point2f(100,300), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255,255));
	 
	
	
	//cv::imshow("Image window", im);
	cv::imshow("Matches", img_matches );
	//std::cout<<"	Matches: "<<matches.size();
	cv::waitKey();
	return matches;
}
