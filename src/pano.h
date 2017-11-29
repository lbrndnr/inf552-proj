#ifndef PANO_H
#define PANO_H

#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

void stitch(Mat I1, Mat I2, Mat H, Mat& K);

#endif