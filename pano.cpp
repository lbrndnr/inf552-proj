#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
using namespace std;
using namespace cv;

const float inlier_threshold = 2.5f; // Distance threshold to identify inliers
const float nn_match_ratio = 0.8f;   // Nearest neighbor matching ratio

int main()
{
	Mat I1 = imread("../IMG_0045.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	Mat I2 = imread("../IMG_0046.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	
	namedWindow("I1", 1);
	namedWindow("I2", 1);
	imshow("I1", I1);
	imshow("I2", I2);
	waitKey(0);

	vector<KeyPoint> m1, m2;
	Mat desc1, desc2;
	Scalar c1(255, 0, 0), c2(0, 255, 0);
	Ptr<AKAZE> D = AKAZE::create();
	
	D->detectAndCompute(I1, noArray(), m1, desc1);
	D->detectAndCompute(I2, noArray(), m2, desc2);

	Mat J, J1, J2;
	drawKeypoints(I1, m1, J1, c1, 0); 
	drawKeypoints(I2, m2, J2, c2, 0);
	hconcat(J1, J2, J);

	namedWindow("J", 1);
	imshow("J", J);
	waitKey(0);

	BFMatcher M(NORM_HAMMING);
    vector< vector<DMatch> > nn_matches;
    M.knnMatch(desc1, desc2, nn_matches, 2);

	vector<KeyPoint> src, dst;

	for(size_t i = 0; i < nn_matches.size(); i++) {
        DMatch first = nn_matches[i][0];
        float dist1 = nn_matches[i][0].distance;
        float dist2 = nn_matches[i][1].distance;

        if(dist1 < nn_match_ratio * dist2) {
            src.push_back(m1[first.queryIdx]);
            dst.push_back(m2[first.trainIdx]);
        }
	}

	Mat Kall;
	drawMatches(J1, src, J2, dst, nn_matches, Kall);
	imshow("K", Kall);
	waitKey(0);	

	vector<Point2f> srcPoints, dstPoints;
	for (size_t i = 0; i < src.size(); i++) {
		srcPoints.push_back(src[i].pt);
		dstPoints.push_back(dst[i].pt);
	}
	
	Mat H = findHomography(srcPoints, dstPoints, RANSAC, 5.0);
	
	Mat K(2 * I1.cols, I1.rows, CV_8U);
	warpPerspective(I1, K, H, Size(K.cols, K.rows), INTER_NEAREST);

	imshow("K", K);
	waitKey(0);
	
	return 0;
}
