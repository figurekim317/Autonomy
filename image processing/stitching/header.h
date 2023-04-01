#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include <set>
#include <vector>
#define INF 1000000000.0f
using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

Mat getTransformationMatrix(vector <KeyPoint> keypoints1, vector <KeyPoint> keypoints2, vector <DMatch> matches);
Mat getStitchedImage(Mat leftImg, Mat rightImg, Mat transformationMatrix);

