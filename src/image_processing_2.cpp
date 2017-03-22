#include "b39vt_assignment/image_processing.hpp"

std::vector<cv::DMatch> templateMatching(const cv::Mat& im, const cv::Mat& sign)
{
	int minHessian = 800;
  std::vector<cv::DMatch> matches;
  cv::Mat descriptors_1, descriptors_2;
  cv::OrbDescriptorExtractor extractor;
  
  cv::OrbFeatureDetector detector(250, 1.2f, 8, 31, 0, 2, 0, 20);

  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    
  detector.detect( sign, keypoints_1 );
  detector.detect( im, keypoints_2 );
  
 
    detector.detect(sign, keypoints_1);
    extractor.compute(sign, keypoints_1, descriptors_1);


  if( !sign.data || !im.data )
   {  std::cout<<"no images loaded"; }
   
  //Calculate descriptors (feature vectors)
  extractor.compute( sign, keypoints_1, descriptors_1 );
  extractor.compute( im, keypoints_2, descriptors_2 );

  //Matching descriptor vectors with a brute force matcher
  cv::BFMatcher matcher(cv::NORM_HAMMING2, true);
  matcher.match( descriptors_1, descriptors_2, matches );
  
  //Quick calculation of max and min distance between keypoints    
  double max_dist = 0; double min_dist = 100;
    
  for( int i = 0; i < matches.size(); i++)
  { double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
  }
  
	std::vector< cv::DMatch >good_matches;
	
  for( int i = 0; i < matches.size(); i++ )
  { 
  	if( matches[i].distance < (max_dist/2) )
    { 
    	good_matches.push_back( matches[i]); 
    }
  }
  
  //Draws good_matches
  cv::Mat img_matches;
  cv::drawMatches( sign, keypoints_1, im, keypoints_2, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

		int i = good_matches.size();
		std::stringstream ss;
    ss << i;



	//outputs number of good_matches to matches window
	cv::putText(img_matches,ss.str(), cv::Point2f(100,460), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255,255));

	cv::imshow("Matches", img_matches );
	cv::waitKey(10);
	
	return good_matches;
}
