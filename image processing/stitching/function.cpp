#include "header.h"

Mat getTransformationMatrix(vector <KeyPoint> keypoints1, vector <KeyPoint> keypoints2, vector <DMatch> matches) {

	srand(time(NULL));
	int iterations = 100;
	Mat A = Mat::zeros(6, 6, CV_32F);
	Mat B(6, 1, CV_32F);
	Mat X(6, 1, CV_32F); // AX = B ���� ���ϰ��� �ϴ� ���
	Mat cur(3, 3, CV_32F);

	Mat output(3, 3, CV_32F); // ��� ���

	set<int> inliers;
	set<int>::iterator it;
	float ax, ay;
	float bx, by;
	float t11;
	float t12;
	float t21;
	float t22;
	float t31;
	float t32;

	float curError;
	float bestError;

	// Tunable variable
	float inlierThreshold = 1.0; // � matching�� inlier�� �Ǳ� ���� ������ ����


	for (int i = 0; i < iterations; i++) {

		inliers.clear();

		// ���Ƿ� ������ 3�� matching ��
		for (int j = 0; j < 3; ) {
			int pairNum = rand() % matches.size();
			if (inliers.find(pairNum) == inliers.end()) {
				inliers.insert(pairNum);
				j++;
			}
		}
		
		//////// �ʱ� ��ȯ��� ���

		// Linear System (AX = B) �� �����ϴ� ��� ���� 
		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
				A.at<float>(i, j) = 0.0f;

		for (int i = 0; i < 6; i++)
			B.at<float>(i, 0) = 0.0f;

		for (it = inliers.begin(); it != inliers.end(); it++) {
			ax = keypoints1[matches[*it].queryIdx].pt.x;
			ay = keypoints1[matches[*it].queryIdx].pt.y;
			bx = keypoints2[matches[*it].trainIdx].pt.x;
			by = keypoints2[matches[*it].trainIdx].pt.y;

			A.at<float>(0, 0) += ax * ax;
			A.at<float>(0, 1) += ax * ay;
			A.at<float>(0, 2) += ax;
			A.at<float>(1, 0) += ax * ay;
			A.at<float>(1, 1) += ay * ay;
			A.at<float>(1, 2) += ay;
			A.at<float>(2, 0) += ax;
			A.at<float>(2, 1) += ay;
			A.at<float>(2, 2) += 1.0f;

			A.at<float>(3, 3) += ax * ax;
			A.at<float>(3, 4) += ax * ay;
			A.at<float>(3, 5) += ax;
			A.at<float>(4, 3) += ax * ay;
			A.at<float>(4, 4) += ay * ay;
			A.at<float>(4, 5) += ay;
			A.at<float>(5, 3) += ax;
			A.at<float>(5, 4) += ay;
			A.at<float>(5, 5) += 1.0f;

			B.at<float>(0, 0) += ax * bx;
			B.at<float>(1, 0) += ay * bx;
			B.at<float>(2, 0) += bx;
			B.at<float>(3, 0) += ax * by;
			B.at<float>(4, 0) += ay * by;
			B.at<float>(5, 0) += by;
		}

		// Linear System ���� ���� ��� X ����
		solve(A, B, X, DECOMP_SVD);

		// ����� X�κ��� �ʱ� ��ȯ��� ����
		cur.at<float>(0, 0) = X.at<float>(0, 0);
		cur.at<float>(0, 1) = X.at<float>(3, 0);
		cur.at<float>(0, 2) = 0.0f;
		cur.at<float>(1, 0) = X.at<float>(1, 0);
		cur.at<float>(1, 1) = X.at<float>(4, 0);
		cur.at<float>(1, 2) = 0.0f;
		cur.at<float>(2, 0) = X.at<float>(2, 0);
		cur.at<float>(2, 1) = X.at<float>(5, 0);
		cur.at<float>(2, 2) = 1.0f;

		// inlier�� ���Ե��� ���� matching�� ��, �ʱ� ��ȯ��Ŀ� ���� ��� ����(inlierThreshold) �̳��� matching�� inlier�� �߰�
		t11 = cur.at<float>(0, 0);
		t12 = cur.at<float>(0, 1);
		t21 = cur.at<float>(1, 0);
		t22 = cur.at<float>(1, 1);
		t31 = cur.at<float>(2, 0);
		t32 = cur.at<float>(2, 1);

		for (int j = 0; j < matches.size(); j++) {
			
			if (inliers.find(j) == inliers.end()) {
			
				float queryX = keypoints1[matches[j].queryIdx].pt.x;
				float queryY = keypoints1[matches[j].queryIdx].pt.y;

				float trainX = keypoints2[matches[j].trainIdx].pt.x;
				float trainY = keypoints2[matches[j].trainIdx].pt.y;

				float transformedX = (t11 * queryX + t12 * queryY + t31);
				float transformedY = (t21 * queryX + t22 * queryY + t32);

				// ���� ���
				float error = (trainX - transformedX)*(trainX - transformedX) + (trainY - transformedY)*(trainY - transformedY);

				if (error < inlierThreshold) inliers.insert(j);
			}
		}

		//////// ��ȯ��� ���

		// Linear System (AX = B) �� �����ϴ� ��� ���� 
		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
				A.at<float>(i, j) = 0.0f;

		for (int i = 0; i < 6; i++)
			B.at<float>(i, 0) = 0.0f;

		for (it = inliers.begin(); it != inliers.end(); it++) {
			ax = keypoints1[matches[*it].queryIdx].pt.x;
			ay = keypoints1[matches[*it].queryIdx].pt.y;
			bx = keypoints2[matches[*it].trainIdx].pt.x;
			by = keypoints2[matches[*it].trainIdx].pt.y;

			A.at<float>(0, 0) += ax * ax;
			A.at<float>(0, 1) += ax * ay;
			A.at<float>(0, 2) += ax;
			A.at<float>(1, 0) += ax * ay;
			A.at<float>(1, 1) += ay * ay;
			A.at<float>(1, 2) += ay;
			A.at<float>(2, 0) += ax;
			A.at<float>(2, 1) += ay;
			A.at<float>(2, 2) += 1.0f;

			A.at<float>(3, 3) += ax * ax;
			A.at<float>(3, 4) += ax * ay;
			A.at<float>(3, 5) += ax;
			A.at<float>(4, 3) += ax * ay;
			A.at<float>(4, 4) += ay * ay;
			A.at<float>(4, 5) += ay;
			A.at<float>(5, 3) += ax;
			A.at<float>(5, 4) += ay;
			A.at<float>(5, 5) += 1.0f;

			B.at<float>(0, 0) += ax * bx;
			B.at<float>(1, 0) += ay * bx;
			B.at<float>(2, 0) += bx;
			B.at<float>(3, 0) += ax * by;
			B.at<float>(4, 0) += ay * by;
			B.at<float>(5, 0) += by;
		}

		// Linear System ���� ������ ��� X ����
		solve(A, B, X, DECOMP_SVD);

		// ����� X�κ��� �ʱ� ��ȯ��� ����
		cur.at<float>(0, 0) = X.at<float>(0, 0);
		cur.at<float>(0, 1) = X.at<float>(3, 0);
		cur.at<float>(0, 2) = 0.0f;
		cur.at<float>(1, 0) = X.at<float>(1, 0);
		cur.at<float>(1, 1) = X.at<float>(4, 0);
		cur.at<float>(1, 2) = 0.0f;
		cur.at<float>(2, 0) = X.at<float>(2, 0);
		cur.at<float>(2, 1) = X.at<float>(5, 0);
		cur.at<float>(2, 2) = 1.0f;

		// ��ȯ����� error ���
		t11 = cur.at<float>(0, 0);
		t12 = cur.at<float>(0, 1);
		t21 = cur.at<float>(1, 0);
		t22 = cur.at<float>(1, 1);
		t31 = cur.at<float>(2, 0);
		t32 = cur.at<float>(2, 1);

		curError = 0.0f;
		for (int j = 0; j < matches.size(); j++) {

			float queryX = keypoints1[matches[j].queryIdx].pt.x;
			float queryY = keypoints1[matches[j].queryIdx].pt.y;

			float trainX = keypoints2[matches[j].trainIdx].pt.x;
			float trainY = keypoints2[matches[j].trainIdx].pt.y;

			float transformedX = (t11 * queryX + t12 * queryY + t31);
			float transformedY = (t21 * queryX + t22 * queryY + t32);

			// ���� ���
			curError += (trainX - transformedX)*(trainX - transformedX) + (trainY - transformedY)*(trainY - transformedY);
		}
		curError /= matches.size();

		// ���� ��ȯ����� ������ �� �۴ٸ� output ������Ʈ
		if (i == 0) {
			output = cur.clone();
			bestError = curError;
		}
		else {
			if (bestError > curError) {
				output = cur.clone();
				bestError = curError;
			}
		}
	}

	return output;
}

Mat getStitchedImage(Mat leftImg, Mat rightImg, Mat transformationMatrix) {

	// leftImg �� ȭ�ҵ��� transformationMatrix(leftImg -> rightImg)�� ��ȯ��
	float t11 = transformationMatrix.at<float>(0, 0);
	float t12 = transformationMatrix.at<float>(0, 1);
	float t21 = transformationMatrix.at<float>(1, 0);
	float t22 = transformationMatrix.at<float>(1, 1);
	float t31 = transformationMatrix.at<float>(2, 0);
	float t32 = transformationMatrix.at<float>(2, 1);

	float minX = 0.0f, maxX = (float)(rightImg.cols-1);
	float minY = 0.0f, maxY = (float)(rightImg.rows-1);
	for (int i = 0; i < leftImg.rows; i++) {
		for (int j = 0; j < leftImg.cols; j++) {
			float x = j, y = i;
			float transformedX = (t11 * x + t12 * y + t31);
			float transformedY = (t21 * x + t22 * y + t32);

			if (minX > transformedX) minX = transformedX;
			if (maxX < transformedX) maxX = transformedX;

			if (minY > transformedY) minY = transformedY;
			if (maxY < transformedY) maxY = transformedY;
		}
	}

	int offsetX, offsetY;
	if (minX < 0.0f) offsetX = -(int)(floor(minX));
	else offsetX = 0;
	if (minY < 0.0f) offsetY = -(int)(floor(minY));
	else offsetY = 0;

	int outputWidth = (int)floor(maxX) + offsetX;
	int outputHeight = (int)floor(maxY) + offsetY;
	printf("output size = %d x %d\n", outputWidth, outputHeight);

	// output image �Ҵ�
	Mat output = Mat::zeros(outputHeight, outputWidth, CV_8UC1);

	// output image ����
	for (int i = 0; i < leftImg.rows; i++) {
		for (int j = 0; j < leftImg.cols; j++) {
			float x = j, y = i;
			float transformedX = (t11 * x + t12 * y + t31);
			float transformedY = (t21 * x + t22 * y + t32);

			int outputY = offsetY + (int)(floor(transformedY));
			int outputX = offsetX + (int)(floor(transformedX));

			if (outputY >= 0 && outputY < outputHeight && outputX >= 0 && outputX < outputWidth)
				output.at<uchar>(outputY, outputX) = leftImg.at<uchar>(i, j);
		}
	}

	for (int i = 0; i < rightImg.rows; i++)
		for (int j = 0; j < rightImg.cols; j++)
			if (offsetY + i < output.rows && offsetX + j < output.cols)
				output.at<uchar>(offsetY + i, offsetX + j) = rightImg.at<uchar>(i, j);

	return output;
}
