#include "pano.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace std;
using namespace cv;

bool isColumnBlack(Mat I, int j) {
    for (int i = 0; i < I.rows; i++) {
        if (I.at<uchar>(i, j) > 0) {
            return false;
        }
    }

    return true;
}

void stitch(Mat I1, Mat I2, Mat H, Mat& K) {
    K = Mat(2 * I1.cols, I1.rows, I1.type());
	warpPerspective(I1, K, Mat::eye(Size(3, 3), CV_32F), Size(I1.cols + I2.cols, I1.rows));
	warpPerspective(I2, K, H, Size(I1.cols + I2.cols, I1.rows), INTER_NEAREST + CV_WARP_INVERSE_MAP, BORDER_TRANSPARENT);

    int i = I1.cols + I2.cols-2;
    while (i > I1.cols + I2.cols/2) {
        if (!isColumnBlack(K, i)) {
            break;
        }
        i--;
    }

    Rect frame(0, 0, i, I1.rows);
    K = K(frame);
}