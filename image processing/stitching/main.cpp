#include "header.h"

int main(int argc, char *argv[]) {

	if (argc != 3) {
		printf("Usage : opencv_stitching.exe [image1 filename] [image2 filename]\n");
		return 1;
	}

	// 이미지 로드
	Mat leftImg = imread(argv[1], IMREAD_GRAYSCALE);
	namedWindow("Left image", WINDOW_AUTOSIZE);
	moveWindow("Left image", 50, 50);
	imshow("Left image", leftImg);
	Mat rightImg = imread(argv[2], IMREAD_GRAYSCALE);
	namedWindow("Right image", WINDOW_AUTOSIZE);
	moveWindow("Right image", 50, 50);
	imshow("Right image", rightImg);

	vector <KeyPoint> leftKeypoints;
	vector <KeyPoint> rightKeypoints;

	Mat leftDescriptors;
	Mat rightDescriptors;

	Ptr <Feature2D> detector = SIFT::create();
	assert(detector != nullptr);
	detector->detectAndCompute(leftImg, Mat(), leftKeypoints, leftDescriptors);
	detector->detectAndCompute(rightImg, Mat(), rightKeypoints, rightDescriptors);

	printf("left keypoints = %d\n", leftKeypoints.size());
	printf("right keypoints = %d\n", rightKeypoints.size());

	// Match
	FlannBasedMatcher matcher;
	vector <DMatch> matches;
	matcher.match(leftDescriptors, rightDescriptors, matches);

	// Extract good matching only
	vector <DMatch> goodMatches;

	double maxDist = 0.0, minDist = 1000000000.0;
	for (int i = 0; i < matches.size(); i++) {
		double curDist = matches[i].distance;
		if (minDist > curDist) minDist = curDist;
		if (maxDist < curDist) maxDist = curDist;
	}

	for (int i = 0; i < matches.size(); i++)
		if (matches[i].distance <= max(2 * minDist, 100.0))
			goodMatches.push_back(matches[i]);

	Mat matchImg;
	drawMatches(leftImg, leftKeypoints, rightImg, rightKeypoints, goodMatches, matchImg, Scalar::all(-1), Scalar::all(-1));
	namedWindow("Matching result", WINDOW_AUTOSIZE);
	moveWindow("Matching result", 50, 50);
	imshow("Matching result", matchImg);

	Mat transformationMatrix = getTransformationMatrix(leftKeypoints, rightKeypoints, goodMatches);
	Mat stitchedImg = getStitchedImage(leftImg, rightImg, transformationMatrix);

	namedWindow("Stitching result", WINDOW_AUTOSIZE);
	moveWindow("Stitching result", 50, 50);
	imshow("Stitching result", stitchedImg);

	waitKey(0);
	destroyWindow("Left image");
	destroyWindow("Right image");
	destroyWindow("Matching result");
	destroyWindow("Stitching result");

	return 0;
}