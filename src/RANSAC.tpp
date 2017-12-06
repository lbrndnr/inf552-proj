using namespace std;
using namespace cv;

struct IterateCondition {

private:
    int maxNumberOfIterations;

public:
    IterateCondition(int i): maxNumberOfIterations(i) {}

	bool operator()(int iterations) const {
		return iterations < maxNumberOfIterations;
	}

};

template <class Data_T>
void selectRandomSubset(vector<Data_T> data, int cardinality, vector<Data_T>& randomSubset) {
    int  m = data.size();
    vector<Data_T> subset;
    vector<int> indices;
    indices.push_back(rand() % m);
    subset.push_back(data[indices[0]]);

    for (int j = 1; j < cardinality; j++) {
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
        subset.push_back(data[indices[j]]);
    }

    randomSubset = subset;
}

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
    assert(minNumberOfDataPoints > 0);
    int maxNumberOfInliers = 0;
    int i = 0;
	int minNumberOfInliers = data.size()*0.1;
	float minError = 20000000;

    while(whileCondition(i)) {
        vector<Data_T> randomSubset;
        selectRandomSubset(data, minNumberOfDataPoints, randomSubset);

        Parameter_T currentParameters;
        calculateParameters(randomSubset, currentParameters);

        int numberOfInliers = 0;
		vector<Data_T> inliers;
		float errorAcumulated = 0.0;
        for (int j = 0; j < data.size(); j++) {
            float error = calculateError(data[j], currentParameters);
            if (error <= errorThreshold) {
				errorAcumulated+=error;
                numberOfInliers++;
				inliers.push_back(data[j]);
            }
        }
        if (numberOfInliers > minNumberOfInliers) {
			errorAcumulated /= numberOfInliers;
			if(maxNumberOfInliers < numberOfInliers){
				maxNumberOfInliers = numberOfInliers;
				minError = errorAcumulated;
				bestFittingParameters = currentParameters;
			} else{
				if(errorAcumulated < minError && maxNumberOfInliers == numberOfInliers){
					maxNumberOfInliers = numberOfInliers;
					minError = errorAcumulated;
					bestFittingParameters = currentParameters;
				}
			}
			
        }
        i++;
    }
}