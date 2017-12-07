#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <queue>
#include <unordered_map>

#include <string>

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
		assert(matches.size() == 4);

        Mat A = Mat::zeros(8, 8, CV_64FC1);
		Mat b(8, 1, CV_64FC1);

		for (int i = 0; i < 4; i++) {
			Point2f p1 = matches[i].first, p2 = matches[i].second;

			int r = 2*i;
			A.at<double>(r, 0) = (double)p2.x;
			A.at<double>(r, 1) = (double)p2.y;
			A.at<double>(r, 2) = (double)1.0;
			A.at<double>(r, 6) = (double)-p2.x*p1.x;
			A.at<double>(r, 7) = (double)-p2.y*p1.x;
			b.at<double>(r, 0) = (double)p1.x;

			r += 1;
			A.at<double>(r, 3) = (double)p2.x;
			A.at<double>(r, 4) = (double)p2.y;
			A.at<double>(r, 5) = (double)1.0;
			A.at<double>(r, 6) = (double)-p2.x*p1.y;
			A.at<double>(r, 7) = (double)-p2.y*p1.y;
			b.at<double>(r, 0) = (double)p1.y;
		}

		Mat H8;
		// Can use solve or can use .inv() and multiply
		//solve(A, b, H8);
		A = A.inv();
		H8 = A * b;

		Mat H = Mat::ones(9, 1, CV_64FC1);
		for (int i = 0; i < 8; i++) {
			H.at<double>(i, 0) = H8.at<double>(i, 0);
		}

		// why 1,3 and not 3,3????
		homography= H.reshape(1, 3);
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

class LinkBetweenPictures {

	public:
		int numberOfMatches_, leftInt_, rightInt_;
		vector<DMatch> matches_;
		Mat leftImage_, rightImage_;
		LinkBetweenPictures(const Mat& leftImage, const Mat& rightImage, int leftInt, int rightInt) {
			if (leftInt == rightInt) {
				cout << "There is a link between the same picture" << endl;
			}
			leftImage_ = leftImage;
			rightImage_ = rightImage;
			leftInt_ = leftInt;
			rightInt_ = rightInt;
			Mat desc1, desc2;
			Ptr<ORB> D = ORB::create(); // TODO do i need to create this each time?
			BFMatcher M(NORM_L2);
			vector<KeyPoint> m1, m2;
			D->detectAndCompute(leftImage, Mat(), m1, desc1);
			D->detectAndCompute(rightImage, Mat(), m2, desc2);
			M.match(desc1, desc2, matches_);
			numberOfMatches_ = matches_.size();
		}
	
};

struct CompareLinks {
	bool operator()(const LinkBetweenPictures& llink, const LinkBetweenPictures& rlink) {
		return llink.numberOfMatches_ < rlink.numberOfMatches_;
	}
};

Mat panorama(const vector<Mat>& pictures) {
	int size = pictures.size();
	Mat image0 = pictures.at(0);
	Mat result = image0;
	vector<Mat> homographies(size);
	Mat H0 = Mat::eye(3, 3, CV_64F);
	homographies[0] = H0;

	Mat canvas = image0;

	for (int i = 1; i < size; i++) {
		//Mat Ii = pictures[i - 1];
		Mat Ii = canvas;
		Mat Ij = pictures[i];

		// Copy paste. TODO Put this on function and re use matches if can.
		//namedWindow("I1", 1);
		//namedWindow("I2", 1);
		//imshow("I1", Ii);
		//imshow("I2", Ij);

		Ptr<AKAZE> D = AKAZE::create();
		vector<KeyPoint> m1, m2;
		Mat desc1, desc2;
		D->detectAndCompute(Ii, Mat(), m1, desc1);
		D->detectAndCompute(Ij, Mat(), m2, desc2);

		Mat J;
		//drawKeypoints(Ii, m1, J, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		//imshow("I1", J);
		//drawKeypoints(Ij, m2, J, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		//imshow("I2", J);
		//waitKey(0);

		BFMatcher M(NORM_L2);
		vector<DMatch> matches;
		M.match(desc1, desc2, matches);
		vector< pair<Point2f, Point2f> > data;
		for (int i = 0; i < matches.size(); i++) {
			data.push_back(make_pair(m1[matches[i].queryIdx].pt, m2[matches[i].trainIdx].pt));
		}
		drawMatches(Ii, m1, Ij, m2, matches, J);
		resize(J, J, Size(), .5, .5);
		//imshow("Matches", J);
		//waitKey(0);
		vector<Point2f> matches1, matches2;
		vector<DMatch> matchesResult;
		float righestMatch = 0;
		
		for (int indexMatches = 0; indexMatches < matches.size(); indexMatches++) {
			float currentX = m1[matches[indexMatches].queryIdx].pt.x;
			if (currentX > righestMatch) {
				righestMatch = currentX;
			}
		}
		for (int indexMatches = 0; indexMatches<matches.size(); indexMatches++) {
			//int distance = abs(m1[matches[indexMatches].queryIdx].pt.x - m2[matches[indexMatches].trainIdx].pt.x);
			float currentX = m1[matches[indexMatches].queryIdx].pt.x;
			if (currentX > righestMatch - (Ij.cols)) {
			//}
			//if ( distance < (Ij.cols*1.5) + (Ii.cols-righestMatch)) {
				matches1.push_back(m1[matches[indexMatches].queryIdx].pt);
				matches2.push_back(m2[matches[indexMatches].trainIdx].pt);
				matchesResult.push_back(matches[indexMatches]);
			}
		}
		// drawMatches(Ii, m1, Ij, m2, matchesResult, J);
		// resize(J, J, Size(), .5, .5);
		// imshow("newMatches!", J);
		// waitKey(0);

		Mat mask; // Inliers?
		Mat Hji = findHomography(matches1, matches2, RANSAC, 3, mask);

		vector<DMatch> inliers;
		for (int i = 0; i<matchesResult.size(); i++)
			if (mask.at<uchar>(i, 0) != 0)
				inliers.push_back(matchesResult[i]);

		drawMatches(Ii, m1, Ij, m2, inliers, J);
		resize(J, J, Size(), .5, .5);
		imshow("newMatches!", J);
		waitKey(0);

		Mat K;
		stitch(Ii, Ij, Hji, K);
//		imshow("Panorama a 2. " + to_string(i-1) + "+" + to_string( i), K);
		//imshow("Panorama a 2.", K);
		//waitKey(0);

		Mat Hi = homographies[i-1];
		Mat Hj = Hi * Hji;
		homographies[i] = Hj;

		canvas = K;
		imwrite("../resources/panoramaResult" + to_string(i) + ".jpg", K);
	}

	return canvas;
}



Mat binaryPanorama(const vector<Mat>& pictures) {
	vector<Mat> currentPictures = pictures, nextPictures;
	bool overlapImages = false;
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

/*
// TODO make it work or drop it
Mat panoramaUnordered(const vector<Mat>& pictures) {
	int size = pictures.size();
	Mat image0 = pictures.at(0);
	priority_queue<LinkBetweenPictures, vector<LinkBetweenPictures>, CompareLinks> pq;
	for (int i = 1; i < size; i++) {
		Mat currentImage = pictures.at(i);
		LinkBetweenPictures currentLink(image0, currentImage, 0, i);
		pq.push(currentLink);
	}

	//unordered_map<Mat, Mat> homographies;
	vector<bool> homographyDone(size);
	vector<Mat> homographies(size);


	Mat H0 = Mat::eye(3, 3, CV_64F);
	homographyDone[0] = true;
	homographies[0] = H0;


	while (!pq.empty()) {
		LinkBetweenPictures currentLink = pq.top();
		pq.pop();
		int leftInt = currentLink.leftInt_;
		int rightInt = currentLink.rightInt_;
		if (homographyDone[leftInt]) {
			Mat Ii = currentLink.leftImage_;
			Mat Ij = currentLink.rightImage_;
			
			// Copy paste. TODO Put this on function and re use matches if can.
			Ptr<ORB> D = ORB::create();
			vector<KeyPoint> m1, m2;
			Mat desc1, desc2;
			D->detectAndCompute(Ii, Mat(), m1, desc1);
			D->detectAndCompute(Ij, Mat(), m2, desc2);
			BFMatcher M(NORM_L2);
			vector<DMatch> matches;
			M.match(desc1, desc2, matches);
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
			Mat Hji = findHomography(matches1, matches2, RANSAC, 3, mask);
			//Mat Hji_V2;
			//ransac(4, data, CalculateHomographyF(), 50.0, CalculateErrorF(), 2000, Hij_V2);
			
			Mat Hi = homographies[leftInt];
			Mat Hj = Hi * Hji;
			homographies[rightInt] = Hj;
			homographyDone[rightInt] = true;

			for (int i = 0; i < size; i++) {
				if (!homographyDone[i] && i!=rightInt) {
					Mat currentImage = pictures.at(i);
					LinkBetweenPictures currentLink(Ij, currentImage, rightInt, i);
					pq.push(currentLink);
				}
			}


		}


	}

	Mat K = image0;
	for (int i = 1; i < size; i++) {
		Mat Kaux;
		Mat I_i = pictures.at(i);
		stitch(K, I_i, homographies[i], Kaux);
		imshow("Panorama", Kaux);
		waitKey(0);
		K = Kaux;

	}

	return K;

	


	
}
*/


int main() {
	vector<Mat> pictures;
	//from 29 to 60 and then 26-28
	for (int i = 29; i <= 60; i++) {
		string fileName = "../resources/pano1/IMG_00" + to_string(i) + ".JPG";
		Mat currentImage = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
		pictures.push_back(currentImage);
	}
	for (int i = 26; i <= 28; i++) {
		string fileName = "../resources/pano1/IMG_00" + to_string(i) + ".JPG";
		Mat currentImage = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
		pictures.push_back(currentImage);
	}
	binaryPanorama(pictures);

	

	Mat I1 = imread("../resources/pano1/IMG_0036.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	Mat I2 = imread("../resources/pano1/IMG_0037.JPG", CV_LOAD_IMAGE_GRAYSCALE);

	//Mat I1 = imread("../resources/IMG_0045.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	//Mat I2 = imread("../resources/IMG_0046.JPG", CV_LOAD_IMAGE_GRAYSCALE);
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
	Mat K1;
	stitch(I1, I2, H, K1);

	imshow("I1+I2 findHomography", K1);
	waitKey(0);


    Mat H2;
   
	
	ransac(4, data, CalculateHomographyF(), 20.0, CalculateErrorF(), 3000, H2);
	cout << H2 << endl;
	// drawMatches(I1, m1, I2, m2, inliers, J);
	// resize(J, J, Size(), .5, .5);
	// imshow("Inliers", J);
	// cout << matches.size() << " matches" << " -> " << inliers.size() << " inliers" << endl;
	// waitKey(0);


	
	Mat K2;
	stitch(I1, I2, H2, K2);
	imshow("I1+I2 2000", K2);
	waitKey(0);

	Mat H3;
	ransac(4, data, CalculateHomographyF(), 20.0, CalculateErrorF(), 1500, H3);
	
	Mat K3;
	stitch(I1, I2, H3, K3);
	imshow("I1+I2 500", K3);
	waitKey(0);

	return 0;
}
