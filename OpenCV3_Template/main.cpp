#include "programs.h"

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
using namespace Programs;

int _num_tracker_max = 500;
int _tracker_slider;

int _hue_max = 255;
int _hue_slider_max;
int _hue_slider_min;

int _saturation_max = 255;
int _saturation_slider_max;
int _saturation_slider_min;

int _value_max = 255;
int _value_slider_max;
int _value_slider_min;


Mat src1;
Mat src2;
Mat dst;


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

	//number of frames til reset opticalflow feature markers
	int FRAME_COUNTER_ORIGINAL = 3;
	int FRAME_COUNTER = FRAME_COUNTER_ORIGINAL;

	Mat current_frame;
	Mat old_frame;

	//Used for feature and corner tracking, contains the grayscaled version of frame
	Mat old_gray;
	Mat current_gray;

	/* Used for tracking sphere*/
	Mat hsv;
	Mat bin;

	//Used for finding contours (the sphere on screen)
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	//points on prev frame and current frame (used for keeping track sphere center and/or scene corners)
	vector<Point2f> points[2];

	int key_pressed = 49;
	/*
	* modes for what type of frame to display
	* 49 -> sphere tracking
	* 50 -> HSV mode
	* 51 -> Binary Image
	* 52 -> Corner Tracking
	*/
	int mode = 49;

	//trackbar for number of features to track
	string num_trackers_title = "Trackers";
	_tracker_slider = 100;
	createTrackbar(num_trackers_title, "Webcam", &_tracker_slider, _num_tracker_max, on_num_trackers_trackbar);
	on_num_trackers_trackbar(_tracker_slider, 0);
	
	//trackbar for hue value MAX
	string num_hue_max_title = "Hue Max";
	_hue_slider_max = 140;
	createTrackbar(num_hue_max_title, "Webcam", &_hue_slider_max, _hue_max, on_num_hue_trackbar);
	on_num_hue_trackbar(_hue_slider_max, 0);
	
	//trackbar for hue value MIN
	string num_hue_min_title = "Hue Min";
	_hue_slider_min = 100;
	createTrackbar(num_hue_min_title, "Webcam", &_hue_slider_min, _hue_max, on_num_hue_trackbar);
	on_num_hue_trackbar(_hue_slider_min, 0);

	
	//trackbar for saturation value MAX
	string num_saturation_max_title = "Saturation Max";
	_saturation_slider_max = 255;
	createTrackbar(num_saturation_max_title, "Webcam", &_saturation_slider_max, _saturation_max, on_num_saturation_trackbar);
	on_num_saturation_trackbar(_saturation_slider_max, 0);

	//trackbar for saturation value MIN
	string num_saturation_min_title = "Saturation  Min";
	_saturation_slider_min = 155;
	createTrackbar(num_saturation_min_title, "Webcam", &_saturation_slider_min, _saturation_max, on_num_saturation_trackbar);
	on_num_saturation_trackbar(_saturation_slider_min, 0);


	//trackbar for value value MAX
	string num_value_max_title = "Value Max";
	_value_slider_max = 255;
	createTrackbar(num_value_max_title, "Webcam", &_value_slider_max, _value_max, on_num_value_trackbar);
	on_num_value_trackbar(_value_slider_max, 0);

	//trackbar for value value MIN
	string num_value_min_title = "Value Min";
	_value_slider_min = 150;
	createTrackbar(num_value_min_title, "Webcam", &_value_slider_min, _value_max, on_num_value_trackbar);
	on_num_value_trackbar(_value_slider_min, 0);


	//trackbar for number of frames
	string num_delay_title = "Delay";
	FRAME_COUNTER_ORIGINAL = 3;
	createTrackbar(num_delay_title, "Webcam", &FRAME_COUNTER_ORIGINAL, 40, on_delay_trackbar);
	on_delay_trackbar(FRAME_COUNTER_ORIGINAL, 0);
	


	//Read first frame
	camera >> old_frame;

	while (1)
	{
		FRAME_COUNTER--;
		//used for calculating optical flow
		vector<uchar> status;
		vector<float> err;
		
		vector<vector<Point> > contours0;
		
		//read the latest frame
		camera >> current_frame;

		if(mode >=49 && mode <= 51) { //for tracking sphere, hsv space, and binary image
			//find contours
			//creating hsv
			cvtColor(current_frame, hsv, COLOR_BGR2HSV);
			//thresholding hsv for blue
			inRange(hsv, Scalar(_hue_slider_min, _saturation_slider_min, _value_slider_min), Scalar(_hue_slider_max, _saturation_slider_max, _value_slider_max), bin);
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
				camera >> old_frame;
				FRAME_COUNTER = FRAME_COUNTER_ORIGINAL;

				//find ball center for the old frame by looking for largest contour
				vector<Point2f> t1; //contain the center of sphere
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

			//find sphere in current frame by looking for largest contour
			vector<Point2f> p1; //will contain center of sphere
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
		}
		else if (mode == 52) { //if user wants to track corners
			
			cvtColor(current_frame, current_gray, COLOR_BGR2GRAY);

			if (FRAME_COUNTER <= 0) {
				FRAME_COUNTER = FRAME_COUNTER_ORIGINAL;
				camera >> old_frame;
				cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
				goodFeaturesToTrack(old_gray, points[0], _tracker_slider, .01, 10, Mat(), 3, 3, 0, 0.04);
				FRAME_COUNTER = FRAME_COUNTER_ORIGINAL;
			}
			goodFeaturesToTrack(current_gray, points[1], _tracker_slider, .01, 10, Mat(), 3, 3, 0, 0.04);
			
		}

		//calculate the optical flow from old frame to current frame
		if (points[0].size() != 0 && points[1].size() != 0) {
			calcOpticalFlowPyrLK(old_frame, current_frame, points[0], points[1], status, err);
			for (int i = 0; i < points[1].size(); i++) {
				line(current_frame, points[1][i], points[0][i], Scalar(0, 255, 0));
				circle(current_frame, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
			}
		}

		
		//Wait for a key to be pressed
		key_pressed = waitKey(5);
		mode = ((key_pressed < 49) || (key_pressed > 53)) ? mode : key_pressed;

		//1 key is pressed
		//show ball track frame
		if(mode == 49)
			imshow("Webcam", current_frame);

		//2 key is pressed
		//show hsv frame
		if (mode == 50)
			imshow("Webcam", hsv);

		//3 key is pressed
		//show binary frame
		if (mode == 51)
			imshow("Webcam", bin);

		//4 key
		//optical flow using corner detection
		if (mode == 52) {
			imshow("Webcam", current_frame);
		}

		//if user presses 5, then program ends
		if (key_pressed == 53) break;
	}

	//Success. The program accomplished its mission and now it can go
	// to the heaven of programs.
	return 0;
}