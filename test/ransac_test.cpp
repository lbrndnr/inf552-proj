#include <iostream>
#include <functional>
#include <random>
#include <fstream>

#include "../src/RANSAC.h"

using namespace std;
using namespace cv;

// A struct representing a line stored in a homogeneous equation (ax + by + c = 0)
struct Line {

	float a, b, c;

	Line(float a, float b, float c): a(a), b(b), c(c) {}
	Line(): a(0), b(0), c(0) {}

};

// This functor computes the line given two points
struct CalculateLineF {

	void operator()(vector<Point2f> const &points, Line &line) const {
		float m = (points[1].y - points[0].y)/(points[1].x - points[0].x);
		float q = points[0].y - points[0].x*m;

		line = Line(m, -1, q);
	}

};

// This functor computes the distance of a point to a line
struct CalculateErrorF {

	float operator()(Point2f p0, Line const &line) const {
		float denum = sqrt(pow(line.a, 2) + pow(line.b, 2));
		return abs(line.a * p0.x + line.b * p0.y + line.c)/denum;
	}

};

// A helper function that returns a float in the range [a, b]
float randomBetween(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

// A helper function that generates a cloud with the a certain amount of outliers
vector<Point2f> generateCloud(int total, float outlierRatio) {
	vector<Point2f> cloud;
	int numberOfInliers = total * (1.0 - outlierRatio);
	int numberOfOutliers = total * outlierRatio;

	// Add inliers
	for (int i = 0; i < numberOfInliers; i++) {
		cloud.push_back(Point2f(i + randomBetween(-5, 5), 2*i + randomBetween(-5, 5)));
	}

	// Add outliers
	for (int i = 0; i < numberOfOutliers; i++) {
		cloud.push_back(Point2f(randomBetween(0, total), randomBetween(0, total)));
	}

	return cloud;
}

// A test function to check how RANSAC behaves for a pseudo random cloud of points
void testRandomRANSAC() {
	cout << "Test RANSAC on a random cloud" << endl;

    vector<Point2f> cloud = generateCloud(1000, 1);

	Line line;
	ransac(2, cloud, CalculateLineF(), 1, CalculateErrorF(), 150, line);
	cout << line.a << "," << line.b << "," << line.c << endl;
}

// A test function to check how RANSAC behaves for a cloud of points 
// with 'total' points and the specified outlier ratio
void testRANSAC(vector<Point2f> cloud, int iterations, string fileName = "") {
	Line line;
	vector<bool> mask;
	ransac(2, cloud, CalculateLineF(), 5, CalculateErrorF(), iterations, line, &mask);
	cout << line.a << "," << line.b << "," << line.c << endl;

	// Write the data cloud to a csv file for the report
	if (!fileName.empty()) {
		ofstream file;
		file.open(fileName);
		file << "x,y,m" << endl;
		for (int i = 0; i < cloud.size(); i++) {
			file << cloud[i].x << "," << cloud[i].y << "," << (mask[i] ? "i" : "o") << endl;
		}
		file.close();
	}
}

int main() {
	srand(time(NULL));

	// testRandomRANSAC();
    // for (int i = 0; i < 10; i++) {
	// 	testRANSAC(1000, (float)i/10.0);
	// }

	vector<Point2f> cloudA = generateCloud(100, 0.6);
	vector<Point2f> cloudB = generateCloud(100, 0.8);

	// First we run it with few iterations
	testRANSAC(cloudA, 10, "../report/plots/ransac1.csv");
	testRANSAC(cloudB, 20, "../report/plots/ransac2.csv");

	// Now we let it run with a bit more iterations
	testRANSAC(cloudA, 20, "../report/plots/ransac3.csv");
	testRANSAC(cloudB, 40, "../report/plots/ransac4.csv");

    return 0;
}