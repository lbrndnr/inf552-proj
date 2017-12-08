#ifndef PANO_H
#define PANO_H

#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

/// Consecutively calls match and stitch to create a panorama image out of I1 and I2
bool matchAndStitch(Mat I1, Mat I2, float overlap, Mat& K, bool shouldShowMatches = false, string fileName = "");

/// Estimates the homography H between I1 and I2
bool match(Mat I1, Mat I2, float overlap, Mat& H, bool shouldShowMatches = false, string fileName = "");
/// Creates a panorama image out of I1 and I2 and crops the black border
void stitch(Mat I1, Mat I2, Mat H, Mat& K);

#endif