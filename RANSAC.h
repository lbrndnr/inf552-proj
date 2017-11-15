#ifndef RANSAC_H
#define RANSAC_H

#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

template <class Parameter_T, class Data_T>
void ransac(int minNumberOfDataPoints,
            vector<Data_T> data,
            std::function<void(vector<Data_T>, Parameter_T&)> calculateParameters, 
            float errorThreshold, 
            std::function<float(Data_T, Parameter_T)> calculateError, 
            Parameter_T& bestFittingParameters,
            std::function<bool(int)> while_condition);

#endif