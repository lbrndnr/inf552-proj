#include "RANSAC.h"

// static void estimateHomography(vector<Keypoint> const &m1, vector<Keypoint> const &m2, Mat &H) {
//     Mat A = zeros();
// }

// static void findHomography(vector<Point2f>& m1, vector<Point2f>& m2, vector<DMatch>& matches) {
//     int m = matches.size();

//     while(true) {
//         int i1 = rand() % m;
//         int i2 = i1, i3 = i1, i4 = i1;
//         while(i2==i1) i2 = rand() % m;
//         while(i3==i1 || i3 == i2) i3 = rand() % m;
//         while(i4==i1 || i4 == i2 || i4 == i3) i4 = rand() % m;


//     }
// }

void calculateLine(vector<Point2f> const &points, vector<float> &line) {
    float m = (points[0].y - points[1].y)/(points[0].x - points[1].x);
    float b = points[0].y - points[0].x*m;

    line.push_back(m);
    line.push_back(b);
}

float calculateError(vector<float> currentParameters, Point2f &point) {
    return 0;
}

void ransac(vector<Point2f> cloud, float errorThreshold, vector<float> &line) {
    int numberOfPoints = 2;
    int m = cloud.size();

    vector<float> bestParameters;
    int maxNumberOfInliers = 0;

    int i = 100;

    while(i >= 0) {
        int i1 = rand() % m;
        int i2 = i1;
        while(i2==i1) i2 = rand() % m;

        vector<float> currentParameters;
        vector<Point2f> currentPoints;
        currentPoints.push_back(cloud[i1]);
        currentPoints.push_back(cloud[i2]);
        calculateLine(currentPoints, currentParameters);

        int numberOfInliers = 0;
        for (int i = 0; i < cloud.size(); i++) {
            float error = calculateError(currentParameters, cloud[i]);
            if (error <= errorThreshold) {
                numberOfInliers++;
            }
        }

        if (numberOfInliers > maxNumberOfInliers) {
            maxNumberOfInliers = numberOfInliers;
            bestParameters = currentParameters;
        }
        i--;
    }


}