#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <queue>
#include <unordered_map>

#include <string>

#include "pano.h"

using namespace std;
using namespace cv;

// void prototype() {
// 	Mat I1 = imread("../resources/tour/IMG_0036.JPG", CV_LOAD_IMAGE_GRAYSCALE);
// 	Mat I2 = imread("../resources/tour/IMG_0037.JPG", CV_LOAD_IMAGE_GRAYSCALE);

// 	//Mat I1 = imread("../resources/IMG_0045.JPG", CV_LOAD_IMAGE_GRAYSCALE);
// 	//Mat I2 = imread("../resources/IMG_0046.JPG", CV_LOAD_IMAGE_GRAYSCALE);
// 	namedWindow("I1", 1);
// 	namedWindow("I2", 1);
// 	imshow("I1", I1);
// 	imshow("I2", I2);

// 	Ptr<ORB> D=ORB::create();

// 	vector<KeyPoint> m1, m2;
// 	Mat desc1, desc2;
// 	D->detectAndCompute(I1, Mat(), m1, desc1);
// 	D->detectAndCompute(I2, Mat(), m2, desc2);

// 	Mat J;
// 	drawKeypoints(I1, m1, J, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
// 	imshow("I1", J);
// 	drawKeypoints(I2, m2, J, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
// 	imshow("I2", J);
// 	waitKey(0);

// 	BFMatcher M(NORM_L2);
// 	vector<DMatch> matches;
// 	M.match(desc1, desc2, matches);

// 	drawMatches(I1, m1, I2, m2, matches, J);
// 	resize(J, J, Size(), .5, .5);
// 	imshow("Matches", J);
// 	waitKey(0);

//     vector< pair<Point2f, Point2f> > data;
// 	for (int i = 0; i < matches.size(); i++) {
// 		data.push_back(make_pair(m1[matches[i].queryIdx].pt, m2[matches[i].trainIdx].pt));
// 	}

// 	vector<Point2f> matches1, matches2;
// 	for (int i = 0; i<matches.size(); i++) {
// 		matches1.push_back(m1[matches[i].queryIdx].pt);
// 		matches2.push_back(m2[matches[i].trainIdx].pt);
// 	}

// 	Mat mask; // Inliers?
// 	Mat H = findHomography(matches1, matches2, RANSAC, 3, mask);

// 		vector<DMatch> inliers;
// 	for (int i = 0; i<matches.size(); i++)
// 		if (mask.at<uchar>(i, 0) != 0)
// 			inliers.push_back(matches[i]);

// 	cout << H << " with " << inliers.size() << " inliers" << endl;
// 	Mat K1;
// 	stitch(I1, I2, H, K1);

//     Mat H2;
   
	
// 	ransac(4, data, CalculateHomographyF(), 20.0, CalculateErrorF(), 3000, H2);
// 	cout << H2 << endl;
// 	// drawMatches(I1, m1, I2, m2, inliers, J);
// 	// resize(J, J, Size(), .5, .5);
// 	// imshow("Inliers", J);
// 	// cout << matches.size() << " matches" << " -> " << inliers.size() << " inliers" << endl;
// 	// waitKey(0);


	
// 	Mat K2;
// 	stitch(I1, I2, H2, K2);
// 	imshow("I1+I2 2000", K2);
// 	waitKey(0);

// 	Mat H3;
// 	ransac(4, data, CalculateHomographyF(), 20.0, CalculateErrorF(), 1500, H3);
	
// 	Mat K3;
// 	stitch(I1, I2, H3, K3);
// 	imshow("I1+I2 500", K3);
// 	waitKey(0);
// }

// Mat naivePanorama(vector<Mat> const &pictures) {
// 	int size = pictures.size();
// 	Mat pano = pictures[0];

// 	for (int i = 1; i < size; i++) {
// 		matchAndStitch(pano, pictures[i], 1.0f, pano, true);
// 	}

// 	return pano;
// }

Mat test(vector<Mat> const &pictures) {
	Mat K;
	matchAndStitch(pictures[0], pictures[1], 1, K, true);
	imshow("K", K);
	waitKey(0);

	return K;
}

Mat binaryPanorama(vector<Mat> const &pictures, bool overlapImages = false) {
	vector<Mat> currentPictures = pictures, nextPictures;
	float overlap = 1.0f;

	while (currentPictures.size() > 1) {
		if (overlapImages) {
			for (int i = 0; i < currentPictures.size()-2; i += 2) {
				Mat I1 = currentPictures[i], I2 = currentPictures[i+1], I3 = currentPictures[i+2];
				Mat K1, K2;
				matchAndStitch(I1, I2, overlap, K1);
				matchAndStitch(K1, I3, overlap/2, K2, true);

				nextPictures.push_back(K2);
			}

			overlapImages = false;
		}
		else {
			for (int i = 0; i < currentPictures.size()-1; i += 2) {
				Mat I1 = currentPictures[i], I2 = currentPictures[i+1];
				Mat K;
				matchAndStitch(I1, I2, overlap, K, true);

				imshow("K", K);
				waitKey(0);

				nextPictures.push_back(K);
			}
		}

		destroyAllWindows();

		overlap /= 2.0f;
		currentPictures = nextPictures;
		nextPictures.clear();
	}

	// imwrite("../resources/panoramaResult" + ".jpg", currentPictures[0]);
	return currentPictures[0];
}

Mat preMatchedPanorama(vector<Mat> const &pictures) {
	Mat pano = pictures[0];
	for (int i = 0; i < pictures.size()-1; i++) {
		Mat H;
		match(pictures[i], pictures[i+1], 1, H, false);
		stitch(pano, pictures[i+1], H, pano);

		imshow("K", pano);
		waitKey(0);
		destroyAllWindows();
	}

	return pano;
}

int main() {
	vector<Mat> pictures;
	for (int i = 30; i <= 60; i++) {
		string fileName = "../resources/tour/IMG_00" + to_string(i) + ".JPG";
		Mat currentImage = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
		pictures.push_back(currentImage);
	}
	binaryPanorama(pictures);
	// test(pictures);

	return 0;
}
