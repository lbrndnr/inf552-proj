#ifndef RANSAC_H
#define RANSAC_H

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <limits>

using namespace cv;
using namespace std;

/// A generic RANSAC algorithm applied on data that chooses the subsets randomly
template <class Parameter_T, class Data_T, class CalculateParameterF, class CalculateErrorF>
void ransac(int minNumberOfDataPoints,
        vector<Data_T> data,
        CalculateParameterF calculateParameters, 
        double errorThreshold, 
        CalculateErrorF calculateError, 
        int maxNumberOfIterations,
        Parameter_T& bestFittingParameters,
        vector<bool>* mask = NULL);

/// A generic RANSAC algorithm applied on data that allows the user to choose a subset himself
/// This is handy as we don't know what kind of data we're dealing with. In some cases it's 
/// more performant to select a 'good' subset.
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