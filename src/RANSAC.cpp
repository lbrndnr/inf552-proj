#include "RANSAC.h"

using namespace cv;
using namespace std;

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

// template <class Parameter_T, class Data_T, class CalculateParameterF>
// void ransac(int minNumberOfDataPoints,
//         vector<Data_T> data,
//         CalculateParameterF calculateParameters, 
//         float errorThreshold, 
//         // std::function<float(Data_T, Parameter_T)> calculateError, 
//         // Parameter_T& bestFittingParameters,
//         std::function<bool(int)> while_condition) {
//     int m = data.size();
//     int maxNumberOfInliers = 0;
//     int i = 0;

//     while(while_condition(i)) {
//         vector<Data_T> currentData;
//         vector<int> indices;
//         indices.push_back(rand() % m);
//         currentData.push_back(data[indices[0]]);

//         for (int j = 1; j < minNumberOfDataPoints; j++) {
//             bool isDiff = false;
//             int currentRandom = 0;
//             while(!isDiff){
//                 currentRandom = rand() % m;
//                 isDiff = true;
//                 for(int k=0; k<j && isDiff;k++) {
//                     if (currentRandom == indices[k]) {
//                         isDiff = false;
//                     }
//                 }
//             }
//             indices.push_back(currentRandom);
//             currentData.push_back(data[indices[j]]);
//         }

//         Parameter_T currentParameters;
//         calculateParameters(currentData, currentParameters);

//         int numberOfInliers = 0;
//         for (int j = 0; j < data.size(); j++) {
//             float error = 0.0f; //calculateError(data[j], currentParameters);
//             if (error <= errorThreshold) {
//                 numberOfInliers++;
//             }
//         }

//         if (numberOfInliers > maxNumberOfInliers) {
//             maxNumberOfInliers = numberOfInliers;
//             // bestFittingParameters = currentParameters;
//         }

//         i++;
//     }


// }

// template <class Parameter_T, Data_T>
// void ransac(int minNumberOfDataPoints,
//             vector<Data_T> cloud,
//             std::function<void(vector<Data_T>, &Parameter_T)> calculateParameters, 
//             float errorThreshold, 
//             std::function<float(Data_T, Parameter_T)> calculateError, 
//             vector<float> &line, 
//             std::function<bool(int)> while_condition) {
//     int numberOfPoints = 2;
//     int m = cloud.size();

//     vector<Point2f> bestPoints;
//     int maxNumberOfInliers = 0;

//     int i = 0;

//     while(while_condition(i)) {
//         int i1 = rand() % m;
//         int i2 = i1;
//         while(i2==i1) i2 = rand() % m;

//         // vector<float> currentParameters;
//         vector<Point2f> currentPoints;
//         currentPoints.push_back(cloud[i1]);
//         currentPoints.push_back(cloud[i2]);
//         // calculateLine(currentPoints, currentParameters);

//         int numberOfInliers = 0;
//         for (int i = 0; i < cloud.size(); i++) {
//             float error = calculateError(currentPoints, cloud[i]);
//             if (error <= errorThreshold) {
//                 numberOfInliers++;
//             }
//         }

//         if (numberOfInliers > maxNumberOfInliers) {
//             maxNumberOfInliers = numberOfInliers;
//             bestPoints = currentPoints;
//         }

//         i++;
//     }


// }