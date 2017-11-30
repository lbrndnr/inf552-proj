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

template <class Data_T>
void selectRandomSubsetA(vector<Data_T> data, int cardinality, vector<Data_T>& randomSubset) {
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

template <class Data_T>
void selectRandomSubsetB(vector<Data_T> data, int cardinality, vector<Data_T>& randomSubset) {
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

struct ChooseSubset {

    template <class Data_T>
	void operator()(vector<Data_T> data, int cardinality, vector<Data_T>& randomSubset) const {
		selectRandomSubsetA(data, cardinality, randomSubset);
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
    ransac(minNumberOfDataPoints, data, calculateParameters, ChooseSubset(), errorThreshold, calculateError, whileCondition, bestFittingParameters);
}

template <class Parameter_T, class Data_T, class ChooseSubsetF, class CalculateParameterF, class IterateConditionF, class CalculateErrorF>
void ransac(int minNumberOfDataPoints,
        vector<Data_T> data,
        CalculateParameterF calculateParameters, 
        ChooseSubsetF chooseSubset,
        double errorThreshold, 
        CalculateErrorF calculateError, 
        IterateConditionF whileCondition,
        Parameter_T& bestFittingParameters) {
    assert(minNumberOfDataPoints > 0);
    int maxNumberOfInliers = 0;
    int i = 0;

    while(whileCondition(i)) {
        vector<Data_T> randomSubset;
        chooseSubset(data, minNumberOfDataPoints, randomSubset);

        Parameter_T currentParameters;
        calculateParameters(randomSubset, currentParameters);

        int numberOfInliers = 0;
        for (int j = 0; j < data.size(); j++) {
            float error = calculateError(data[j], currentParameters);
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