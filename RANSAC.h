#ifndef RANSAC_H
#define RANSAC_H

#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

// template <typename T> class RANSAC {
	
// };

void ransac(vector<Point2f> cloud, float errorThreshold, vector<float> &line);

#endif