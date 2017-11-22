#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <functional>

#include "RANSAC.h"

using namespace std;
using namespace cv;

bool iterate_while(int i) {
	return i < 100;
}

struct CalculateLine {
	void operator()(vector<Point2f> const &points, vector<Point2f> &line) const {
		line = points;
	}
};

struct CalculateError {
	float operator()(Point2f p0, vector<Point2f> &currentParameters) const {
		Point2f p1 = currentParameters[0];
		Point2f p2 = currentParameters[1];

		float dy = p2.y-p1.y;
		float dx = p2.x - p1.x;

		return abs(dy * p0.x - dx * p0.y + p2.x * p1.y + p2.y * p1.x)/sqrt(dy*dy + dx*dx);
	}
};

// void calculateLine(vector<Point2f> const &points, vector<float> &line) {
//     float m = (points[0].y - points[1].y)/(points[0].x - points[1].x);
//     float b = points[0].y - points[0].x*m;

//     line.push_back(m);
//     line.push_back(b);
// }

int main()
{
	vector<Point2f> cloud;
	cloud.push_back(Point2f(1, 1));
	cloud.push_back(Point2f(2, 2));

	vector<Point2f> line;
	ransac(2, cloud, CalculateLine(), 1, CalculateError(), &iterate_while, line);
	cout << line << endl;

	Mat I1 = imread("../resources/IMG_0045.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	Mat I2 = imread("../resources/IMG_0046.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	//Mat I2 = imread("../IMG_0046r.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	namedWindow("I1", 1);
	namedWindow("I2", 1);
	imshow("I1", I1);
	imshow("I2", I2);

	Ptr<ORB> D=ORB::create();
	//Ptr<AKAZE> D = AKAZE::create();

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

	BFMatcher M(NORM_L2/*,true*/);
	vector<DMatch> matches;
	M.match(desc1, desc2, matches);
	cout << matches.size() << endl;

	drawMatches(I1, m1, I2, m2, matches, J);
	resize(J, J, Size(), .5, .5);
	imshow("Matches", J);
	waitKey(0);

	vector<Point2f> matches1, matches2;
	for (int i = 0; i<matches.size(); i++) {
		matches1.push_back(m1[matches[i].queryIdx].pt);
		matches2.push_back(m2[matches[i].trainIdx].pt);
	}

	Mat mask; // Inliers?
	Mat H = findHomography(matches1, matches2, RANSAC, 3, mask);
	cout << H << endl;
	vector<DMatch> inliers;
	for (int i = 0; i<matches.size(); i++)
		if (mask.at<uchar>(i, 0) != 0)
			inliers.push_back(matches[i]);

	drawMatches(I1, m1, I2, m2, inliers, J);
	resize(J, J, Size(), .5, .5);
	imshow("Inliers", J);
	cout << matches.size() << " matches" << " -> " << inliers.size() << " inliers" << endl;
	waitKey(0);

	Mat K(2 * I1.cols, I1.rows, CV_8U);
	warpPerspective(I1, K, Mat::eye(Size(3, 3), CV_32F), Size(2 * I1.cols, I1.rows));
	warpPerspective(I2, K, H, Size(2 * I1.cols, I1.rows), CV_INTER_LINEAR + CV_WARP_INVERSE_MAP, BORDER_TRANSPARENT);
	imshow("I1+I2", K);


	waitKey(0);
	return 0;
}
