#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "RANSAC.h"
#include "pano.h"

using namespace std;
using namespace cv;

Mat genMatFromPoint(Point2f p) {
	Mat A = Mat::ones(3, 1, CV_64F);
	A.at<double>(0, 0) = (double)p.x;
    A.at<double>(1, 0) = (double)p.y;

	return A;
}

struct CalculateHomographyF {

	void operator()(vector< pair<Point2f, Point2f> > const &matches, Mat &homography) const {
        // Mat P = Mat::zeros(2*matches.size(), 9, CV_32FC1);
		// Mat b = Mat::zeros(2*matches.size(), 1, CV_32FC1);
		// Mat H;

		// for (int i = 0; i < matches.size(); i++) {
		// 	Point2f p1 = matches[i].first, p2 = matches[i].second;

		// 	int r = 2*i;
		// 	P.at<float>(r, 0) = -p1.x;
		// 	P.at<float>(r, 1) = -p1.y;
		// 	P.at<float>(r, 2) = -1.0;
		// 	P.at<float>(r, 6) = p1.x*p2.x;
		// 	P.at<float>(r, 7) = p1.y*p2.x;
		// 	P.at<float>(r, 8) = p2.x;

		// 	r += 1;
		// 	P.at<float>(r, 3) = -p1.x;
		// 	P.at<float>(r, 4) = -p1.y;
		// 	P.at<float>(r, 5) = -1.0;
		// 	P.at<float>(r, 6) = p1.x*p2.y;
		// 	P.at<float>(r, 7) = p1.y*p2.x;
		// 	P.at<float>(r, 8) = p2.y;
		// }

		// solve(P.t()*P, P.t()*b, H, DECOMP_SVD);

		// H = H.reshape(1, 3);
		// cout << H << endl;

		vector<Point2f> src, dst;

		for (unsigned int i = 0; i < matches.size(); i++){
			src.push_back(matches[i].first);
			dst.push_back(matches[i].second);
		}

		Mat H = findHomography(dst, src);

		homography = H;
	}

};

struct CalculateErrorF {

	float operator()(pair<Point2f, Point2f> match, Mat &H) const {
        Mat A = genMatFromPoint(match.first);
		Mat B = genMatFromPoint(match.second);
        Mat X = H*B;

		X /= X.at<double>(2, 0);

        return norm(X, A, NORM_L2);
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
		data.push_back(make_pair(m1[matches[i].queryIdx].pt, m2[matches[i].trainIdx].pt));
	}

	vector<Point2f> matches1, matches2;
	for (int i = 0; i<matches.size(); i++) {
		matches1.push_back(m1[matches[i].queryIdx].pt);
		matches2.push_back(m2[matches[i].trainIdx].pt);
	}

	Mat mask; // Inliers?
	Mat H = findHomography(matches1, matches2, RANSAC, 3, mask);

		vector<DMatch> inliers;
	for (int i = 0; i<matches.size(); i++)
		if (mask.at<uchar>(i, 0) != 0)
			inliers.push_back(matches[i]);

	cout << H << " with " << inliers.size() << " inliers" << endl;

    // Mat H;
    ransac(4, data, CalculateHomographyF(), 20.0, CalculateErrorF(), 2000, H);
	
	// why is this needed?
	H = H.inv();
	cout << H << endl;

	// drawMatches(I1, m1, I2, m2, inliers, J);
	// resize(J, J, Size(), .5, .5);
	// imshow("Inliers", J);
	// cout << matches.size() << " matches" << " -> " << inliers.size() << " inliers" << endl;
	// waitKey(0);

	Mat K;
	stitch(I1, I2, H, K);

	imshow("I1+I2", K);
	waitKey(0);

	return 0;
}
