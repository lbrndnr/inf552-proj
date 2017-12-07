#include "pano.h"
#include "RANSAC.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// Helper function to generate a matrix out of a 2d point
Mat genMatFromPoint(Point2f p) {
	Mat A = Mat::ones(3, 1, CV_64F);
	A.at<double>(0, 0) = (double)p.x;
    A.at<double>(1, 0) = (double)p.y;

	return A;
}

// The default functor we use to calculate the homography
struct CalculateHomographyF {

	void operator()(vector< pair<Point2f, Point2f> > const &matches, Mat &homography) const {
		assert(matches.size() == 4);

        // First we build the system of linear equations to find the homography H that maps points
        // Note that A is only 8x8 instead of 9x8 as we expect H_33=1
        // Source: http://www.csc.kth.se/~perrose/files/pose-init-model/node17.html
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

        // Solve the system of equations
		Mat H8;
		solve(A, b, H8);

        // We create a column vector of ones and copy H8 into it
        // This is the same effect as appending 1 to the column vector H8
		Mat H = Mat::ones(9, 1, CV_64FC1);
		H8.copyTo(H(Rect(0, 0, 1, 8)));

		// Reshaping the matrix to make a 3x3 matrix out of a column vector
		homography= H.reshape(1, 3);
	}

};

// The default functor that we use to estimate the error
struct CalculateErrorF {

	float operator()(pair<Point2f, Point2f> match, Mat &H) const {
        Mat A = genMatFromPoint(match.first);
		Mat B = genMatFromPoint(match.second);
        Mat X = H*B;

		X /= X.at<double>(2, 0);

        return norm(X, A, NORM_L2);
	}

};

// An advanced functor to choose a 'random' subset more efficiently
// The idea was inspired by OpenCV: We don't choose the subset not entirely randomly
// so that the RANSAC algorithm needs less iterations in order to find a good subset
// to estimate the model
struct ChooseGoodSubsetF {

    // This function checks whether there are any two points in the subset that
    // are basically on the same line. If there are, the subset is not very adequate
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
        int iterations = 0;

		int j = 1;
		while (j < cardinality) {
            iterations++;
            
            // Choose a random, distinct pair and add it to the subset
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

            // We only do this for 1000 iterations, otherwise we give up to avoid infinite loops
			if (iterations < 1000) {
                // Check if the left or right subset has two points on the same line
                if (!this->checkSubset(firstSubset) || !this->checkSubset(secondSubset)) {
                    // If one of the does, we remove one random pair and try again
                    int idx = rand() % j;
                    indices.erase(indices.begin() + idx);
                    firstSubset.erase(firstSubset.begin() + idx);
                    secondSubset.erase(secondSubset.begin() + idx);

                    continue;
                }
            }

			j++;
		}

        // Merge the two separate subsets to a vector of pairs again
		vector< pair<Point2f, Point2f> > subset;
		for (int i = 0; i < cardinality; i++) {
			subset.push_back(make_pair(firstSubset[i], secondSubset[i]));
		}

        randomSubset = subset;
	}

};

// Returns true if the column j in image I is black
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
    // We use AKAZE instead of ORB as it yielded way better results (better matches)
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
    float rightMostMatch = 0;
    
    // Find the right most match to estimate where the image begins and where the black border starts
    for (int indexMatches = 0; indexMatches < matches.size(); indexMatches++) {
        float currentX = m1[matches[indexMatches].queryIdx].pt.x;
        if (currentX > rightMostMatch) {
            rightMostMatch = currentX;
        }
    }

    // Now all the matches are filtered to only include matches at most (overlap * I2.cols) away from the right edge of the left image
    // This makes sure that no key points in the right image are matched with key points that actually exist (possible matches)
    for (int indexMatches = 0; indexMatches<matches.size(); indexMatches++) {
        float currentX = m1[matches[indexMatches].queryIdx].pt.x;
        if (currentX > rightMostMatch - I2.cols*overlap) {
            data.push_back(make_pair(m1[matches[indexMatches].queryIdx].pt, m2[matches[indexMatches].trainIdx].pt));
            matchesResult.push_back(matches[indexMatches]);
        }
    }

    vector<bool> mask;
    // H = findHomography(matches1, matches2, RANSAC, 10, mask);
    ransac(4, data, CalculateHomographyF(), ChooseGoodSubsetF(), 5.0, CalculateErrorF(), 2000, H, (shouldDrawMatches) ? &mask : NULL);
    H = H.inv();

    if (shouldDrawMatches) {
        // Filter all matches to find the inliers according to our RANSAC algorithm
        vector<DMatch> inliers;
        for (int i = 0; i < matchesResult.size(); i++) {
            if (mask[i]) {
                inliers.push_back(matchesResult[i]);
            }
        }

        Mat J;
        drawMatches(I1, m1, I2, m2, inliers, J);
        resize(J, J, Size(), .5, .5);
        imshow("Inlining matches", J);
        waitKey(0);
    }
}

void stitch(Mat I1, Mat I2, Mat H, Mat& K) {
    // K is about twice as large I1 => leaves a border on the right side as I2 will be distorted
    K = Mat(I1.cols + I2.cols, I1.rows, I1.type());
	warpPerspective(I1, K, Mat::eye(Size(3, 3), CV_32F), Size(I1.cols + I2.cols, I1.rows));
	warpPerspective(I2, K, H, Size(I1.cols + I2.cols, I1.rows), INTER_NEAREST + CV_WARP_INVERSE_MAP, BORDER_TRANSPARENT);

    // Figure out how wide the border is on the right side
    int i = I1.cols + I2.cols-2;
    while (i > I1.cols + I2.cols/2) {
        if (!isColumnBlack(K, i)) {
            break;
        }
        i--;
    }

    // Cut the border off
    Rect frame(0, 0, i, I1.rows);
    K = K(frame);
}