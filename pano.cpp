#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

#include "RANSAC.h"

using namespace std;
using namespace cv;

int main()
{
	Mat I1 = imread("../IMG_0045.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	Mat I2 = imread("../IMG_0046.JPG", CV_LOAD_IMAGE_GRAYSCALE);

	Ptr<AKAZE> D = AKAZE::create();

	vector<KeyPoint> m1, m2;
	Mat desc1, desc2;
	D->detectAndCompute(I1, Mat(), m1, desc1);
	D->detectAndCompute(I2, Mat(), m2, desc2);

	BFMatcher M(NORM_L2);
	vector<DMatch> matches;
	M.match(desc1, desc2, matches);

	vector<Point2f> cloud;
	cloud.push_back(Point2f(1, 1));
	cloud.push_back(Point2f(2, 4));

	vector<float> line;
	ransac(cloud, 1.0f, line);
	cout << "m: " << line[0] << "b: " << line[1];
	system("pause");
	return 0;
}
