#include "pano.h"
#include "RANSAC.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

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

struct ChooseGoodSubsetF {

	bool checkSubset(vector<Point2f> const &points) const {
		const float threshold = 0.996f;
		int m = points.size();
		for (int j = 0; j < m-1; j++) {
			Point2f d1 = points[j]-points[m-1];
			float n1 = d1.x * d1.x + d1.y * d1.y;

			for (int k = 0; k < j; k++) {
				Point2f d2 = points[k]-points[m-1];
				float denom = (d2.x*d2.x + d2.y*d2.y)*n1;
				float num = d1.x*d2.x + d1.y*d2.y;

				if (num*num > threshold*threshold*denom) {
					return false;
				}
			}
		}

		return true;
	}

	void operator()(vector< pair<Point2f, Point2f> > const &data, int cardinality, vector< pair<Point2f, Point2f> >& randomSubset) const {
        int  m = data.size();
        vector<Point2f> firstSubset;
		vector<Point2f> secondSubset;
        vector<int> indices;
        indices.push_back(rand() % m);
        firstSubset.push_back(data[indices[0]].first);
		secondSubset.push_back(data[indices[0]].second);

		int j = 1;
		while (j < cardinality) {
			bool isDiff = false;
            int currentRandom = 0;
            while(!isDiff){
                currentRandom = rand() % m;
                isDiff = true;
                for(int k=0; k<j && isDiff;k++) {
                    if (currentRandom == indices[k]) {
                        isDiff = false;
                    }
                }
            }
            indices.push_back(currentRandom);
            firstSubset.push_back(data[indices[j]].first);
			secondSubset.push_back(data[indices[j]].second);

			if (!this->checkSubset(firstSubset) || !this->checkSubset(secondSubset)) {
				int idx = rand() % j;
				indices.erase(indices.begin() + idx);
				firstSubset.erase(firstSubset.begin() + idx);
				secondSubset.erase(secondSubset.begin() + idx);

				continue;
			}

			j++;
		}

		vector< pair<Point2f, Point2f> > subset;
		for (int i = 0; i < cardinality; i++) {
			subset.push_back(make_pair(firstSubset[i], secondSubset[i]));
		}

        randomSubset = subset;
	}

};

bool isColumnBlack(Mat I, int j) {
    for (int i = 0; i < I.rows; i++) {
        if (I.at<uchar>(i, j) > 0) {
            return false;
        }
    }

    return true;
}

void matchAndStitch(Mat I1, Mat I2, float overlap, Mat& K, bool shouldDrawMatches) {
    Mat H;
    match(I1, I2, overlap, H, shouldDrawMatches);
    stitch(I1, I2, H, K);
}

void match(Mat I1, Mat I2, float overlap, Mat& H, bool shouldDrawMatches) {
    Ptr<AKAZE> D = AKAZE::create();
    vector<KeyPoint> m1, m2;
    Mat desc1, desc2;
    D->detectAndCompute(I1, Mat(), m1, desc1);
    D->detectAndCompute(I2, Mat(), m2, desc2);

    BFMatcher M(NORM_L2);
    vector<DMatch> matches;
    M.match(desc1, desc2, matches);

    vector< pair<Point2f, Point2f> > data;
    vector<DMatch> matchesResult;
    float righestMatch = 0;
    
    for (int indexMatches = 0; indexMatches < matches.size(); indexMatches++) {
        float currentX = m1[matches[indexMatches].queryIdx].pt.x;
        if (currentX > righestMatch) {
            righestMatch = currentX;
        }
    }
    for (int indexMatches = 0; indexMatches<matches.size(); indexMatches++) {
        float currentX = m1[matches[indexMatches].queryIdx].pt.x;
        if (currentX > righestMatch - I2.cols*overlap) {
            data.push_back(make_pair(m1[matches[indexMatches].queryIdx].pt, m2[matches[indexMatches].trainIdx].pt));
            matchesResult.push_back(matches[indexMatches]);
        }
    }

    Mat mask; // Inliers?
    // H = findHomography(matches1, matches2, RANSAC, 10, mask);
    ransac(4, data, CalculateHomographyF(), ChooseGoodSubsetF(), 5.0, CalculateErrorF(), 2000, H);
    H = H.inv();

    if (shouldDrawMatches) {
        // vector<DMatch> inliers;
        // for (int i = 0; i<matchesResult.size(); i++)
        //     if (mask.at<uchar>(i, 0) != 0)
        //         inliers.push_back(matchesResult[i]);

        Mat J;
        drawMatches(I1, m1, I2, m2, matches, J);
        resize(J, J, Size(), .5, .5);
        imshow("Inlining matches", J);
        waitKey(0);
    }
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