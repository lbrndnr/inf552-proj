#include <ctime>
// TODO

using namespace cv;
using namespace std;

struct IterateCondition {

private:
    int maxNumberOfIterations;

public:
    IterateCondition(int i): maxNumberOfIterations(i) {}

	bool operator()(int iterations) const {
		return iterations < maxNumberOfIterations;
	}

};

template <class Parameter_T, class Data_T, class CalculateParameterF, class CalculateErrorF>
void ransac(int minNumberOfDataPoints,
        vector<Data_T> data,
        CalculateParameterF calculateParameters, 
        double errorThreshold, 
        CalculateErrorF calculateError, 
        int maxNumberOfIterations,
        Parameter_T& bestFittingParameters) {
    ransac(minNumberOfDataPoints, data, calculateParameters, errorThreshold, calculateError, IterateCondition(maxNumberOfIterations), bestFittingParameters);
}

template <class Parameter_T, class Data_T, class CalculateParameterF, class IterateConditionF, class CalculateErrorF>
void ransac(int minNumberOfDataPoints,
        vector<Data_T> data,
        CalculateParameterF calculateParameters, 
        double errorThreshold, 
        CalculateErrorF calculateError, 
        IterateConditionF whileCondition,
        Parameter_T& bestFittingParameters) {
    int m = data.size();
    int maxNumberOfInliers = 0;
    int i = 0;

	double elapsed_secs, elapsed_secs_rand =0, elapsed_secs_homography=0, elapsed_secs_error=0;
	clock_t begin, end;

    while(whileCondition(i)) {
        vector<Data_T> currentData;
        vector<int> indices;
        indices.push_back(rand() % m);
        currentData.push_back(data[indices[0]]);

		
		begin = clock();
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
		end = clock();
		elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		elapsed_secs_rand+=elapsed_secs;

		
		begin = clock();
        Parameter_T currentParameters;
        calculateParameters(currentData, currentParameters);
		end = clock();
		elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		elapsed_secs_homography+=elapsed_secs;
		
		begin = clock();
        int numberOfInliers = 0;
        for (int j = 0; j < data.size(); j++) {
            float error = calculateError(data[j], currentParameters);
            if (error <= errorThreshold) {
                numberOfInliers++;
            }
        }
		end = clock();
		elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		elapsed_secs_error+=elapsed_secs;

        if (numberOfInliers >= maxNumberOfInliers) {
            maxNumberOfInliers = numberOfInliers;
            bestFittingParameters = currentParameters;
        }

        i++;
    }
	cout << "Elapsed time with rand" << elapsed_secs_rand << endl;
	cout << "Elapsed time with homography" << elapsed_secs_homography << endl;
	cout << "Elapsed time with checking error" << elapsed_secs_error << endl;
}