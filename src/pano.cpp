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
    for (int i = 0; i < matches.size(); i++) {
        data.push_back(make_pair(m1[matches[i].queryIdx].pt, m2[matches[i].trainIdx].pt));
    }

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
        if (currentX > righestMatch - I2.cols*overlap) {
        //}
        //if ( distance < (Ij.cols*1.5) + (Ii.cols-righestMatch)) {
            matches1.push_back(m1[matches[indexMatches].queryIdx].pt);
            matches2.push_back(m2[matches[indexMatches].trainIdx].pt);
            matchesResult.push_back(matches[indexMatches]);
        }
    }

    Mat mask; // Inliers?
    H = findHomography(matches1, matches2, RANSAC, 10, mask);

    if (shouldDrawMatches) {
        vector<DMatch> inliers;
        for (int i = 0; i<matchesResult.size(); i++)
            if (mask.at<uchar>(i, 0) != 0)
                inliers.push_back(matchesResult[i]);

        Mat J;
        drawMatches(I1, m1, I2, m2, inliers, J);
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