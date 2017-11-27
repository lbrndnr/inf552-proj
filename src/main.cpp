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
	Mat A = Mat(3, 1, CV_32F);
	A.at<float>(0, 0) = p.x;
    A.at<float>(1, 0) = p.y;

	return A;
}

struct CalculateHomographyF {

	void operator()(vector< pair<Point2f, Point2f> > const &matches, Mat &homography) const {
        // Mat A = Mat::ones(matches.size(), 3, CV_32FC1);
        // Mat B = Mat::ones(matches.size(), 3, CV_32FC1);
        // Mat H;

        // for (int i = 0; i < matches.size(); i++) {
        //     A.at<float>(i, 0) = matches[i].first.x/800.0;
        //     A.at<float>(i, 1) = matches[i].first.y/533.0;

        //     B.at<float>(i, 0) = matches[i].second.x/800.0;
        //     B.at<float>(i, 1) = matches[i].second.y/533.0;
        // }

        // solve(A.t()*A, A.t()*B, H, DECOMP_CHOLESKY);

        // homography = H;

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
        // Mat A = genMatFromPoint(match.first);

		// cout << A << endl;
        // Mat X = H*A;

		// cout << A << endl << genMatFromPoint(match.second) << endl;

        // return norm(A-genMatFromPoint(match.second));

		Point2f point1 = match.first;
		Point2f point2 = match.second;
		float sub = (float) (H.at<double>(2, 0)*point2.x + H.at<double>(2, 1)*point2.y + H.at<double>(2, 2));

		if (sub == 0.){
			//cout << "Point a l'infini !" << endl;
			return DBL_MAX;
		}

		//H ne peut etre accedee qu'en double, il faut donc cast pour avoir des floats
		Point2f imp2(
				(float)(H.at<double>(0, 0)*point2.x + H.at<double>(0, 1)*point2.y + H.at<double>(0, 2)) / sub,
				(float)(H.at<double>(1, 0)*point2.x + H.at<double>(1, 1)*point2.y + H.at<double>(1, 2)) / sub
		);

		return pow(point1.x-imp2.x,2)+pow(point1.y-imp2.y,2);
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

	Mat K(2 * I1.cols, I1.rows, CV_8U);
	warpPerspective(I1, K, Mat::eye(Size(3, 3), CV_32F), Size(2 * I1.cols, I1.rows));
	warpPerspective(I2, K, H, Size(2 * I1.cols, I1.rows), CV_INTER_LINEAR + CV_WARP_INVERSE_MAP, BORDER_TRANSPARENT);
	imshow("I1+I2", K);
	waitKey(0);

	return 0;
}
