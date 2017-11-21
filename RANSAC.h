#ifndef RANSAC_H
#define RANSAC_H

#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

template <class Parameter_T, class Data_T, class CalculateParameterF>
void ransac(int minNumberOfDataPoints,
        vector<Data_T> data,
        CalculateParameterF calculateParameters, 
        float errorThreshold, 
        // std::function<float(Data_T, Parameter_T)> calculateError, 
        std::function<bool(int)> while_condition,
        Parameter_T& bestFittingParameters) {
    int m = data.size();
    int maxNumberOfInliers = 0;
    int i = 0;

    while(while_condition(i)) {
        vector<Data_T> currentData;
        vector<int> indices;
        indices.push_back(rand() % m);
        currentData.push_back(data[indices[0]]);

        for (int j = 1; j < minNumberOfDataPoints; j++) {
            bool isDiff = false;
            int currentRandom = 0;
            while(!isDiff){
                currentRandom = rand() % m;
                isDiff = true;
                for(int k=0; k<j && isDiff;k++) {
                    if (currentRandom == indices[k]) {
                        isDiff = false;
                    }
                }
            }
            indices.push_back(currentRandom);
            currentData.push_back(data[indices[j]]);
        }

        Parameter_T currentParameters;
        calculateParameters(currentData, currentParameters);

        int numberOfInliers = 0;
        for (int j = 0; j < data.size(); j++) {
            float error = 0.0f; //calculateError(data[j], currentParameters);
            if (error <= errorThreshold) {
                numberOfInliers++;
            }
        }

        if (numberOfInliers > maxNumberOfInliers) {
            maxNumberOfInliers = numberOfInliers;
            bestFittingParameters = currentParameters;
        }

        i++;
    }
}


#endif