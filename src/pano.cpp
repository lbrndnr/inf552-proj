#include "pano.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace std;
using namespace cv;

void stitch(Mat I1, Mat I2, Mat H, Mat& K) {
	// K = Mat(I1.rows, I1.cols + I2.cols, I1.type());

    // Mat area1 = K(Rect(0,0,I1.cols,I2.rows));
    // I1.copyTo(area1);

    // I2 = H * I2;
    // Mat area2 = K(Rect(I1.cols,0,I2.cols,I2.rows));
    // I2.copyTo(area2);

    K = Mat(2 * I1.cols, I1.rows, I1.type());
	warpPerspective(I1, K, Mat::eye(Size(3, 3), CV_32F), Size(20 * I2.cols, I1.rows));
	warpPerspective(I2, K, H, Size(20 * I2.cols, I1.rows), INTER_NEAREST + CV_WARP_INVERSE_MAP, BORDER_TRANSPARENT);
}