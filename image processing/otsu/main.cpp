#include "header.h"

int main(void) {

	std::cout << "OpenCV Version : " << CV_VERSION << std::endl;

	cv::Mat img;
	img = cv::imread("mandrill.pgm", CV_LOAD_IMAGE_GRAYSCALE);
	if (img.empty()) {
		std::cout << "Invalid filename" << std::endl;
		return -1;
	}

	// Show original image
	cv::namedWindow("Original Image", CV_WINDOW_AUTOSIZE);
	cv::imshow("Original Image", img);
	cv::waitKey(0);
	cv::destroyWindow("Original Image");

	// Make histogram of image
	int histogram[256] = { 0, };
	getHistogram(img, histogram);

	// Run Otsu's algorithm
	int threshold = otsu(histogram, 256);
	std::cout << "Threshold = " << threshold << std::endl;

	// Binarize image
	binarize(img, threshold);

	// Show binarized image
	cv::namedWindow("Binarized Image", CV_WINDOW_AUTOSIZE);
	cv::imshow("Binarized Image", img);
	cv::waitKey(0);
	cv::destroyWindow("Binarized Image");

	return 0;
}
