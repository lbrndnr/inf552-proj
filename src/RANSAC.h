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
        Parameter_T& bestFittingParameters,
        vector<bool>* mask = NULL);

template <class Parameter_T, class Data_T, class ChooseSubsetF, class CalculateParameterF, class CalculateErrorF>
void ransac(int minNumberOfDataPoints,
        vector<Data_T> data,
        CalculateParameterF calculateParameters, 
        ChooseSubsetF chooseSubset,
        double errorThreshold, 
        CalculateErrorF calculateError, 
        int maxNumberOfIterations,
        Parameter_T& bestFittingParameters,
        vector<bool>* mask = NULL);

// In order to separate the template declaration from its implementation
#include "RANSAC.tpp"

#endif