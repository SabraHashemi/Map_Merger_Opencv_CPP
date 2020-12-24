
#include "mapmerge.h"
#include "math.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <cmath>
# define M_PI           3.14159265358979323846
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>      //for imshow
#include <vector>
#include <iostream>
#include <iomanip>


using namespace cv;


StitchedMap::StitchedMap(Mat &img1, Mat &img2, float max_pairwise_distance)
{
  // load images, TODO: check that they're grayscale
  image1 = img1.clone();
  image2 = img2.clone();

  if (image1.size != image2.size)
         cv::resize(image2, image2, image1.size());
  std::vector<cv::KeyPoint> keyPoints1, keyPoints2;
  Ptr<FeatureDetector> detector = ORB::create();
  detector->detect(image1,  keyPoints1);
  detector->detect(image2, keyPoints2);


  // 2. extract descriptors
  cv::Mat descriptors1, descriptors2;
  Ptr<DescriptorExtractor> descriptorExtractor = ORB::create();
  
  descriptorExtractor->compute(image1, keyPoints1, descriptors1);
  descriptorExtractor->compute(image2, keyPoints2, descriptors2);


  // Match features.
  std::vector<cv::DMatch> matches;
  FlannBasedMatcher matcher;
  BFMatcher dematc(NORM_HAMMING, false);
  dematc.match(descriptors1, descriptors2, matches);

  //if (descriptors1.type() != CV_32F) {
  //    descriptors1.convertTo(descriptors1, CV_32F);
  //}

  //if (descriptors2.type() != CV_32F) {
  //    descriptors2.convertTo(descriptors2, CV_32F);
  //}
  //matcher.match(descriptors1, descriptors2, matches);
 //matchFeatures(descriptors1, descriptors2, matches);
  // Draw matches.
  cv::Mat image_matches;
  cv::drawMatches(image1, keyPoints1, image2, keyPoints2, matches, image_matches);
  cv::imshow("hhh", image_matches);
  cv::waitKey(0);







  ////Matching descriptor vectors using FLANN matcher
  //FlannBasedMatcher matcher2;
  //std::vector< DMatch > matches2;
  //matcher.match(descriptors1, descriptors2, matches2);
  //cout << "number of matches (FLANN): " << matches2.size() << endl;

  //// Matching descriptor vectors using Brute Force matcher
  //BFMatcher BFmatcher(NORM_L2);
  //vector<DMatch> BFmatches;
  //BFmatcher.match(descriptors1, descriptors2, BFmatches);

  //cout << "number of matches (Brute Force): " << BFmatches.size() << endl;

  // 4. find matching point pairs with same distance in both images
  for (size_t i=0; i<matches.size(); i++) {
    KeyPoint a1 = keyPoints1[matches[i].queryIdx],b1 = keyPoints2[matches[i].trainIdx];

    if (matches[i].distance > 30)
      continue;

    for (size_t j=0; j<matches.size(); j++) {
      KeyPoint a2 = keyPoints1[matches[j].queryIdx],
               b2 = keyPoints2[matches[j].trainIdx];

      if (matches[j].distance > 30)
        continue;

      if ( fabs(norm(a1.pt-a2.pt) - norm(b1.pt-b2.pt)) > max_pairwise_distance ||
           fabs(norm(a1.pt-a2.pt) - norm(b1.pt-b2.pt)) == 0)
        continue;

      coord1.push_back(a1.pt);
      coord1.push_back(a2.pt);
      coord2.push_back(b1.pt);
      coord2.push_back(b2.pt);

      fil1.push_back(a1);
      fil1.push_back(a2);
      fil2.push_back(b1);
      fil2.push_back(b2);
    }
  }

  if (coord1.size() == 0)
    ;
  // 5. find homography
  H=estimateAffinePartial2D(coord1, coord2);
  //H = estimateRigidTransform(coord2, coord1, false);
  // 6. calculate this stuff for information
  rotation = 180./M_PI*atan2(H.at<double>(0,1),H.at<double>(1,1)),
  transx   = H.at<double>(0,2),
  transy   = H.at<double>(1,2);
  scalex   = sqrt(pow(H.at<double>(0,0),2)+pow(H.at<double>(0,1),2));
  scaley   = sqrt(pow(H.at<double>(1,0),2)+pow(H.at<double>(1,1),2));
}

Mat
StitchedMap::get_debug()
{
  Mat out;
  drawKeypoints(image1, kpv1, image1, Scalar(255,0,0));
  drawKeypoints(image2, kpv2, image2, Scalar(255,0,0));
  drawMatches(image1,fil1, image2,fil2, matches,out,Scalar::all(-1),Scalar::all(-1));
  return out;
}

Mat // return the stitched maps
StitchedMap::get_stitch()
{
  // create storage for new image and get transformations
  Mat image(image2.size(), image2.type());
  warpAffine(image2,image,H,image.size());

  // blend image1 onto the transformed image2
  addWeighted(image,.5,image1,.5,0.0,image);

  return image;
}

StitchedMap::~StitchedMap() { }
