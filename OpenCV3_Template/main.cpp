#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;


int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;

Mat src1;
Mat src2;
Mat dst;

//edit trackbar
static void on_trackbar(int, void*)
{
	alpha = (double)alpha_slider / alpha_slider_max;
	beta = (1.0 - alpha);
	addWeighted(src1, alpha, src2, beta, 0.0, dst);
	imshow("Linear Blend", dst);
}


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
	namedWindow("Linear Blend", WINDOW_AUTOSIZE); // Create Window
	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
	//dnamedWindow("bin", CV_WINDOW_AUTOSIZE);

	//number of frames til reseting opticalflow feature markers
	int FRAME_COUNTER_ORIGINAL = 3;
	int FRAME_COUNTER = FRAME_COUNTER_ORIGINAL;

	Mat current_frame;
	Mat old_frame;
	//Mat old_gray;
	//Mat current_gray;

	//edit trackbar
	camera >> src1;
	camera >> src2;
	char TrackbarName[50];
	alpha_slider = 0;
	createTrackbar(TrackbarName, "Linear Blend", &alpha_slider, alpha_slider_max, on_trackbar);
	on_trackbar(alpha_slider, 0);
	
	//points on prev frame and current frame
	vector<Point2f> points[2];

	//Read first frame
	camera >> old_frame;
	//cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	while (1)
	{
		FRAME_COUNTER--;
		vector<uchar> status;
		vector<float> err;

		//read the latest frame
		camera >> current_frame;
		//cvtColor(current_frame, current_gray, COLOR_BGR2GRAY);
		//find contours
		vector<vector<Point> > contours0;
		Mat hsv;
		Mat bin;
		//creating hsv
		cvtColor(current_frame, hsv, COLOR_BGR2HSV);
		//thresholding hsv for blue
		inRange(hsv, Scalar(100, 150, 0), Scalar(140, 255, 255), bin);
		//remove noise
		erode(bin, bin, Mat(), Point(-1, -1), 2);
		dilate(bin, bin, Mat(), Point(-1, -1), 2);
		//find contours in filtered binary picture
		findContours(bin, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		contours.resize(contours0.size());
		//Find poly curves with given contours
		for (size_t k = 0; k < contours0.size(); k++)
			approxPolyDP(Mat(contours0[k]), contours[k], 3, true);

		//find sphere in old frame
		if (FRAME_COUNTER <= 0) {
			camera >> src2;
			//edit trackbar
			camera >> old_frame;
			//cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
			//goodFeaturesToTrack(old_gray, points[0], 500, .01, 10, Mat(), 3, 3, 0, 0.04);
			FRAME_COUNTER = FRAME_COUNTER_ORIGINAL;

			//find ball center for the old frame
			vector<Point2f> t1;
			if(contours.size() > 0) {
				int max = 0;
				int max_ind = -1;
				vector<Point> largest_contour;
				for (int i = 0; i < contours.size(); i++) {
					if (contours[i].size() > max) {
						max = contours[i].size();
						max_ind = i;
					}
				}
				largest_contour = contours[max_ind];
				Point2f cent;
				float rad = 0;
				minEnclosingCircle(largest_contour, cent, rad);
				if(rad > 10) {
					circle(current_frame, cent, rad, Scalar(10, 255, 0));
					circle(current_frame, cent, 3, Scalar(10, 255, 255), 5);
				}
				t1.push_back(cent);
			}
			points[0] = t1;
		}

		//find sphere in current frame
		vector<Point2f> p1;
		int max = 0;
		int max_ind = -1;
		if (contours.size() > 0) {
			vector<Point> largest_contour;
			for (int i = 0; i < contours.size(); i++) {
				if (contours[i].size() > max) {
					max = contours[i].size();
					max_ind = i;
				}
			}
			largest_contour = contours[max_ind];
			Point2f cent;
			float rad = 0;
			minEnclosingCircle(largest_contour, cent, rad);
			circle(current_frame, cent, rad, Scalar(10, 255, 0));
			circle(current_frame, cent, 3, Scalar(10, 255, 255),5);
			p1.push_back(cent);
		}
		points[1] = p1;
		

		//calculate the optical flow from old frame to current frame
		if (points[0].size() != 0 && points[1].size() != 0) {
			calcOpticalFlowPyrLK(old_frame, current_frame, points[0], points[1], status, err);
			for (int i = 0; i < points[1].size(); i++) {
				line(current_frame, points[1][i], points[0][i], Scalar(0, 255, 0));
				circle(current_frame, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
			}
		}

		//draw cicle on features and lines from old frame to current frame
		imshow("Webcam", current_frame);
		//imshow("bin", bin);
		//Wait for a key to be pressed
		if (waitKey(30) >= 0) break;
	}

	//Success. The program accomplished its mission and now it can go
	// to the heaven of programs.
	return 0;
}