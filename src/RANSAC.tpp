using namespace std;
using namespace cv;

// Default functor that chooses a random subset
struct ChooseSubsetF {

    template <class Data_T>
	void operator()(vector<Data_T> const &data, int cardinality, vector<Data_T>& randomSubset) const {
        int  m = data.size();
        vector<Data_T> subset;
        vector<int> indices;
        indices.push_back(rand() % m);
        subset.push_back(data[indices[0]]);

        for (int j = 1; j < cardinality; j++) {
            // We do this in order to assure that we use distinct data points
            // Otherwise, calculating a model is not possible in most cases
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

};

template <class Parameter_T, class Data_T, class CalculateParameter_F, class CalculateError_F>
void ransac(int minNumberOfDataPoints,
        vector<Data_T> data,
        CalculateParameter_F calculateParameters, 
        double errorThreshold, 
        CalculateError_F calculateError, 
        int maxNumberOfIterations,
        Parameter_T& bestFittingParameters,
        vector<bool>* mask) {
    ransac(minNumberOfDataPoints, data, calculateParameters, ChooseSubsetF(), errorThreshold, calculateError, maxNumberOfIterations, bestFittingParameters, mask);
}

template <class Parameter_T, class Data_T, class ChooseSubset_F, class CalculateParameter_F, class CalculateError_F>
void ransac(int minNumberOfDataPoints,
        vector<Data_T> data,
        CalculateParameter_F calculateParameters, 
        ChooseSubset_F chooseSubset,
        double errorThreshold, 
        CalculateError_F calculateError, 
        int maxNumberOfIterations,
        Parameter_T& bestFittingParameters,
        vector<bool>* mask) {
    // First, we make sure that the supplied arguments make sense
    assert(minNumberOfDataPoints > 0);
    assert(data.size() > minNumberOfDataPoints);

    int maxNumberOfInliers = 0;

    for (int i = 0; i < maxNumberOfIterations; i++) {
        // We select a subset according to the functor supplied. This is random by default but doesn't have to be.
        vector<Data_T> subset;
        chooseSubset(data, minNumberOfDataPoints, subset);

        // We estimate the current model for the chosen subset
        Parameter_T currentParameters;
        calculateParameters(subset, currentParameters);

        // We iterate over all data points, compute the error if we used the current model and count the inliers
        int numberOfInliers = 0;
        for (int j = 0; j < data.size(); j++) {
            float error = calculateError(data[j], currentParameters);
            if (error <= errorThreshold) {
                numberOfInliers++;
            }
        }

        // It's the best model if it has the most inliers so far
        if (numberOfInliers > maxNumberOfInliers) {
            maxNumberOfInliers = numberOfInliers;
            bestFittingParameters = currentParameters;			
        }
    }

    // If we want to know the inliers we iterate of the data points once more to distinguish outliers from inliers
    if (mask != NULL) {
        for (int j = 0; j < data.size(); j++) {
            float error = calculateError(data[j], bestFittingParameters);
            mask->push_back(error <= errorThreshold);
        }
    }
}