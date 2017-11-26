#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "RANSAC.h"
#include "pano.h"

using namespace std;
using namespace cv;

struct CalculateHomographyF {

	void operator()(vector< pair<Point2f, Point2f> > const &matches, Mat &homography) const {
        Mat A = Mat::zeros(matches.size(), 2, CV_32FC1);
        Mat B = Mat::zeros(matches.size(), 2, CV_32FC1);
        Mat H;

        for (int i = 0; i < matches.size(); i++) {
            A.at<float>(i, 0) = matches[i].first.x;
            A.at<float>(i, 1) = matches[i].first.y;

            B.at<float>(i, 0) = matches[i].second.x;
            B.at<float>(i, 1) = matches[i].second.y;
        }

        solve(A.t()*A, A.t()*B, H, DECOMP_CHOLESKY);
        homography = H;
	}

};

struct CalculateErrorF {

	float operator()(pair<Point2f, Point2f> match, Mat &H) const {
        Mat A = Mat(match.first).t();
        Mat X = A * H;

        return norm(A-Mat(match.second).t());
	}

};

int main() {
	Mat I1 = imread("../resources/IMG_0045.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	Mat I2 = imread("../resources/IMG_0046.JPG", CV_LOAD_IMAGE_GRAYSCALE);

	namedWindow("I1", 1);
	namedWindow("I2", 1);
	imshow("I1", I1);
	imshow("I2", I2);

	Ptr<ORB> D=ORB::create();

	vector<KeyPoint> m1, m2;
	Mat desc1, desc2;
	D->detectAndCompute(I1, Mat(), m1, desc1);
	D->detectAndCompute(I2, Mat(), m2, desc2);

	Mat J;
	drawKeypoints(I1, m1, J, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imshow("I1", J);
	drawKeypoints(I2, m2, J, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imshow("I2", J);
	waitKey(0);

	BFMatcher M(NORM_L2);
	vector<DMatch> matches;
	M.match(desc1, desc2, matches);

	drawMatches(I1, m1, I2, m2, matches, J);
	resize(J, J, Size(), .5, .5);
	imshow("Matches", J);
	waitKey(0);

    vector< pair<Point2f, Point2f> > data;
	for (int i = 0; i < matches.size(); i++) {
		data.push_back(make_pair(m1[matches[i].trainIdx].pt, m2[matches[i].imgIdx].pt));
	}

    Mat H;
    ransac(4, data, CalculateHomographyF(), 3.0, CalculateErrorF(), 10000, H);

	// Mat mask; // Inliers?
	// Mat H = findHomography(matches1, matches2, RANSAC, 3, mask);
	// cout << H << endl;
	// vector<DMatch> inliers;
	// for (int i = 0; i<matches.size(); i++)
	// 	if (mask.at<uchar>(i, 0) != 0)
	// 		inliers.push_back(matches[i]);

	// drawMatches(I1, m1, I2, m2, inliers, J);
	// resize(J, J, Size(), .5, .5);
	// imshow("Inliers", J);
	// cout << matches.size() << " matches" << " -> " << inliers.size() << " inliers" << endl;
	// waitKey(0);

	// Mat K(2 * I1.cols, I1.rows, CV_8U);
	// warpPerspective(I1, K, Mat::eye(Size(3, 3), CV_32F), Size(2 * I1.cols, I1.rows));
	// warpPerspective(I2, K, H, Size(2 * I1.cols, I1.rows), CV_INTER_LINEAR + CV_WARP_INVERSE_MAP, BORDER_TRANSPARENT);
	// imshow("I1+I2", K);

	// waitKey(0);
	return 0;
}
