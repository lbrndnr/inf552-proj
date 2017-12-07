#include <iostream>
#include <functional>
#include <random>
#include <fstream>

#include "../src/RANSAC.h"

using namespace std;
using namespace cv;

struct Line {

	float a, b, c;

	Line(float a, float b, float c): a(a), b(b), c(c) {}
	Line(): a(0), b(0), c(0) {}

};

struct CalculateLine {
	void operator()(vector<Point2f> const &points, Line &line) const {
		float m = (points[1].y - points[0].y)/(points[1].x - points[0].x);
		float q = points[0].y - points[0].x*m;

		line = Line(m, -1, q);
	}
};

struct CalculateError {
	float operator()(Point2f p0, Line const &line) const {
		float denum = sqrt(pow(line.a, 2) + pow(line.b, 2));
		return abs(line.a * p0.x + line.b * p0.y + line.c)/denum;
	}
};

float randomBetween(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

void testRandomRANSAC() {
	cout << "Test RANSAC on a random cloud" << endl;

    vector<Point2f> cloud;
	int total = 1000;

	// Add outliers
	for (int i = 0; i < total; i++) {
		cloud.push_back(Point2f(randomBetween(0, total), randomBetween(0, total)));
	}

	Line line;
	ransac(2, cloud, CalculateLine(), 1, CalculateError(), 150, line);
	cout << line.a << "," << line.b << "," << line.c << endl;
}

void testRANSAC(int total, float outlierRatio, string fileName = "") {
	vector<Point2f> cloud;
	int numberOfInliers = total * (1.0 - outlierRatio);
	int numberOfOutliers = total * outlierRatio;

	cout << "Test RANSAC with " << numberOfOutliers << " outliers and " << numberOfInliers << " inliers"  << endl;


	// Add inliers
	for (int i = 0; i < numberOfInliers; i++) {
		cloud.push_back(Point2f(i, i));
	}

	// Add outliers
	for (int i = 0; i < numberOfOutliers; i++) {
		cloud.push_back(Point2f(randomBetween(0, total), randomBetween(0, 5)));
	}

	Line line;
	ransac(2, cloud, CalculateLine(), 1, CalculateError(), 150, line);
	cout << line.a << "," << line.b << "," << line.c << endl;

	if (!fileName.empty()) {
		ofstream file;
		file.open(fileName);
		file << "x,y" << endl;
		for (int i = 0; i < cloud.size(); i++) {
			file << cloud[i].x << "," << cloud[i].y << endl;
		}
		file.close();
	}
}

int main() {
	testRandomRANSAC();
    for (int i = 0; i < 10; i++) {
		testRANSAC(1000, (float)i/10.0);
	}

	testRANSAC(100, 0.2, "../report/plots/ransac1.csv");

    return 0;
}