#include <opencv2/imgproc/imgproc.hpp>

#include "RANSAC.cpp"

// template void ransac(int minNumberOfDataPoints,
//         vector<cv::Point2f> data,
//         // ParameterCalculatorF calculateParameters, 
//         float errorThreshold, 
//         // std::function<float(Data_T, Parameter_T)> calculateError, 
//         // Parameter_T& bestFittingParameters,
//         std::function<bool(int)> while_condition);


// template void ransac(int minNumberOfDataPoints,
//         vector<cv::Point2f> data,
//         CalculateParameterF calculateParameters, 
//         float errorThreshold, 
//         // std::function<float(Data_T, Parameter_T)> calculateError, 
//         // Parameter_T& bestFittingParameters,
//         std::function<bool(int)> while_condition);