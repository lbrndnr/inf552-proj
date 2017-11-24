#ifndef RANSAC_H
#define RANSAC_H

#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <limits>

using namespace cv;
using namespace std;

template <class Parameter_T, class Data_T, class CalculateParameterF, class CalculateErrorF>
void ransac(int minNumberOfDataPoints,
        vector<Data_T> data,
        CalculateParameterF calculateParameters, 
        double errorThreshold, 
        CalculateErrorF calculateError, 
        int maxNumberOfIterations,
        Parameter_T& bestFittingParameters);

template <class Parameter_T, class Data_T, class CalculateParameterF, class IterateConditionF, class CalculateErrorF>
void ransac(int minNumberOfDataPoints,
        vector<Data_T> data,
        CalculateParameterF calculateParameters, 
        double errorThreshold, 
        CalculateErrorF calculateError, 
        IterateConditionF whileCondition,
        Parameter_T& bestFittingParameters);

// In order to separate the template declaration from its implementation
#include "RANSAC.tpp"

#endif