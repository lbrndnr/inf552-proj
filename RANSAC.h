#ifndef RANSAC_H
#define RANSAC_H

#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

template <class Parameter_T, class Data_T, class Func_Calculate_Param, class Func_Calculate_Error, class Allocator>
void ransac(int minNumberOfDataPoints,
            vector<Data_T, Allocator> data,
			Func_Calculate_Param calculateParameters,
            float errorThreshold, 
			Func_Calculate_Error calculateError,
            Parameter_T& bestFittingParameters,
			int numOfIterations);

#endif