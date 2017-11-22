#include <iostream>
#include <functional>

#include "../src/RANSAC.h"

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

void testRANSAC() {
    vector<Point2f> cloud;
	cloud.push_back(Point2f(1, 1));
	cloud.push_back(Point2f(2, 2));

	vector<Point2f> line;
	ransac(2, cloud, CalculateLine(), 1, CalculateError(), &iterate_while, line);
	cout << line << endl;
}

int main() {
    cout << "Test RANSAC" << endl;
    testRANSAC();

    return 0;
}