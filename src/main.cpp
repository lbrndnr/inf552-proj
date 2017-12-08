#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <queue>
#include <unordered_map>

#include <string>

#include "pano.h"

using namespace std;
using namespace cv;

// Just a test function for debugging
Mat test(vector<Mat> const &pictures) {
	Mat K;
	matchAndStitch(pictures[13], pictures[14], 1, K, true, "../resources/matchers/akaze1.jpg");
	imshow("K", K);
	waitKey(0);
	imwrite("../resources/matchers/akaze1_pano.jpg", K);

	return K;
}

// Creates one single image out of the pictures array
// Works by first stitching single images two a pair, then pairs to an image of 4 and so on
// To reduce the number of stitching compared to the naive way => reduces errors 
Mat binaryPanorama(vector<Mat> const &pictures, bool overlapImages = false) {
	vector<Mat> currentPictures = pictures, nextPictures;
	float overlap = 1.0f;
	int k = 0;
	bool stitchingPossible = true;

	while (currentPictures.size() > 1 && stitchingPossible) {
		// If we overlap images we stitch 3 images together in the first round instead of only two
		// Thus, if we continue matching, we'd basically match the same image on both sides
		// The idea was that if we have more key points => more matches => better result
		// Didn't really work in practice though
		if (overlapImages) {
			for (int i = 0; i < currentPictures.size()-2; i += 2) {
				Mat I1 = currentPictures[i], I2 = currentPictures[i+1], I3 = currentPictures[i+2];
				Mat K1, K2;
				matchAndStitch(I1, I2, overlap, K1);
				matchAndStitch(K1, I3, overlap/2, K2, true);

				nextPictures.push_back(K2);
			}

			overlapImages = false;
		}
		else {
			cout << currentPictures.size() << " pictures left to stitch" << endl;
			for (int i = 0; i < currentPictures.size()-1; i += 2) {
				string fileName = "../resources/output/binary_panorama" + to_string(k) + ".jpg";
				cout << "Stitching " << fileName << endl;

				Mat I1 = currentPictures[i], I2 = currentPictures[i+1];
				Mat K;
				stitchingPossible = matchAndStitch(I1, I2, overlap, K, false, fileName);
				if (!stitchingPossible) {
					cout << "Aborting. Stitching not possible anymore." << endl;
					break;
				}

				// imshow("K", K);
				// waitKey(0);

				k++;
				nextPictures.push_back(K);
			}

			// Add remaining image so that we don't lose it
			if ((currentPictures.size() % 2) == 1) {
				nextPictures.push_back(currentPictures[currentPictures.size()-1]);
			}
		}

		destroyAllWindows();

		// We always divide the overlapping width by two such that
		// we really only look for matches that make sense
		overlap /= 2.0f;
		currentPictures = nextPictures;
		nextPictures.clear();
	}

	return currentPictures[0];
}

// Creates one single image out of the pictures array
// Works by first calculating the homography of very pair (without stitching them together)
// And in the second stage stitching them together once every pair has been matched
// Doesn't work as the homography changes along with the pictures being stitched together
Mat preMatchedPanorama(vector<Mat> const &pictures) {
	Mat pano = pictures[0];
	for (int i = 0; i < pictures.size()-1; i++) {
		Mat H;
		match(pictures[i], pictures[i+1], 1, H, false);
		stitch(pano, pictures[i+1], H, pano);

		imshow("K", pano);
		waitKey(0);
		destroyAllWindows();
	}

	return pano;
}

int main() {
	// Loading the pictures
	// Note that we rearranged the sequence of images such that the order
	// corresponds to a panorama as usually taken in practice
	vector<Mat> pictures;
	for (int i = 30; i <= 60; i++) {
		string fileName = "../resources/tour/IMG_00" + to_string(i) + ".JPG";
		Mat currentImage = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
		pictures.push_back(currentImage);
	}
	binaryPanorama(pictures);
	// test(pictures);

	return 0;
}
