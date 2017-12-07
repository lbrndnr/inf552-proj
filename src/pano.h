#ifndef PANO_H
#define PANO_H

#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

/// Consecutively calls match and stitch to create a panorama image out of I1 and I2
void matchAndStitch(Mat I1, Mat I2, float overlap, Mat& K, bool shouldDrawMatches = false);

/// Estimates the homography H between I1 and I2
void match(Mat I1, Mat I2, float overlap, Mat& H, bool shouldDrawMatches = false);
/// Creates a panorama image out of I1 and I2 and crops the black border
void stitch(Mat I1, Mat I2, Mat H, Mat& K);

#endif