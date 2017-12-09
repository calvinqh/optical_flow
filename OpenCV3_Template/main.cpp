#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int, char**)
{
	//Open the first webcam plugged in the computer
	VideoCapture camera(0);
	if (!camera.isOpened())
	{
		cerr << "ERROR: Could not open camera" << endl;
		return 1;
	}

	//Create a window to display the images from the webcam
	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

	//number of frames til reseting opticalflow feature markers
	int FRAME_COUNTER = 0; 

	Mat current_frame;
	Mat old_frame;

	Mat old_gray;
	Mat current_gray;
	
	//points on prev frame and current frame
	vector<Point2f> points[2];

	//Read first frame
	camera >> old_frame;
	cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
	goodFeaturesToTrack(old_gray, points[0], 500, .01, 10, Mat(), 3, 3, 0, 0.04);



	while (1)
	{
		FRAME_COUNTER++;
		vector<uchar> status;
		vector<float> err;

		//reset good features markers
		if (FRAME_COUNTER > 20) {
			camera >> old_frame;
			cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
			goodFeaturesToTrack(old_gray, points[0], 500, .01, 10, Mat(), 3, 3, 0, 0.04);
			FRAME_COUNTER = 0;
		}

		

		//find the features of the new frame
		camera >> current_frame;
		cvtColor(current_frame, current_gray, COLOR_BGR2GRAY);
		goodFeaturesToTrack(current_gray, points[1], 500, .01, 10, Mat(), 3, 3, 0, 0.04);

		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		int _levels = 3 - 3;

		vector<vector<Point> > contours0;
		Mat hsv;
		Mat bin;
		cvtColor(current_frame, hsv, COLOR_BGR2HSV);
		inRange(hsv, Scalar(100,150,0), Scalar(140,255,255),bin);

		findContours(bin, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		contours.resize(contours0.size());
		for (size_t k = 0; k < contours0.size(); k++)
			approxPolyDP(Mat(contours0[k]), contours[k], 3, true);
		drawContours(current_frame, contours, _levels <= 0 ? 3 : -1, Scalar(128, 255, 255), 3, LINE_AA, hierarchy, std::abs(_levels));
		/*
		vector<Vec3f> circles;
		//detect circles
		Mat smooth_current_gray;
		GaussianBlur(current_gray, smooth_current_gray, Size(9, 9), 2, 2);
		HoughCircles(smooth_current_gray, circles, HOUGH_GRADIENT, 2, smooth_current_gray.rows / 4, 200, 100);
		for (size_t i = 0; i < circles.size(); i++) {
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			circle(current_frame, center, 3, Scalar(255, 0, 0), -1, 8, 0);
			circle(current_frame, center, radius, Scalar(0, 0, 255), 3, 8, 0);
		}
		*/
		//calculate the optical flow from old frame to current frame
		calcOpticalFlowPyrLK(old_frame, current_frame, points[0], points[1], status, err);

		//draw cicle on features and lines from old frame to current frame
		/*
		for (int i = 0; i < points[1].size(); i++) {
			line(current_frame, points[1][i], points[0][i], Scalar(0, 255, 0));
			circle(current_frame, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
		}
		*/
		//Show the image on the window
		imshow("Webcam", current_frame);

		//Wait for a key to be pressed
		if (waitKey(30) >= 0) break;
	}

	//Success. The program accomplished its mission and now it can go
	// to the heaven of programs.
	return 0;
}