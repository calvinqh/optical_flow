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


int _num_features_max = 100;
int _feature_slider;
double alpha;
double beta;

int _hue_max = 255;
int _hue_slider;

int _saturation_max = 255;
int _saturation_slider;

int _value_max = 255;
int _value_slider;


Mat src1;
Mat src2;
Mat dst;

//edit trackbar
static void on_num_trackers_trackbar(int, void*)
{
	/*
	alpha = (double)_feature_slider / _num_features_max;
	beta = (1.0 - alpha);
	addWeighted(src1, alpha, src2, beta, 0.0, dst);
	*/
	//imshow("Linear Blend", dst);
}

static void on_num_hue_trackbar(int, void*)
{
	/*
	alpha = (double)_hue_slider / _hue_max;
	beta = (1.0 - alpha);
	addWeighted(src1, alpha, src2, beta, 0.0, dst);
	//imshow("Linear Blend", dst);
	*/
}

static void on_num_saturation_trackbar(int, void*)
{
	/*
	alpha = (double)_saturation_slider / _saturation_max;
	beta = (1.0 - alpha);
	addWeighted(src1, alpha, src2, beta, 0.0, dst);
	//imshow("Linear Blend", dst);
	*/
}

static void on_num_value_trackbar(int, void*)
{
	/*
	alpha = (double)_value_slider / _value_max;
	beta = (1.0 - alpha);
	addWeighted(src1, alpha, src2, beta, 0.0, dst);
	//imshow("Linear Blend", dst);
	*/
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
	//namedWindow("Linear Blend", WINDOW_AUTOSIZE); // Create Window
	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
	//dnamedWindow("bin", CV_WINDOW_AUTOSIZE);

	//number of frames til reseting opticalflow feature markers
	int FRAME_COUNTER_ORIGINAL = 3;
	int FRAME_COUNTER = FRAME_COUNTER_ORIGINAL;

	Mat current_frame;
	Mat old_frame;

	/*Used for feature / corner tracking*/
	Mat old_gray;
	Mat current_gray;

	camera >> src1;
	camera >> src2;

	//trackbar for number of features to track
	string num_trackers_title = "Trackers";
	_feature_slider = 100;
	createTrackbar(num_trackers_title, "Webcam", &_feature_slider, _num_features_max, on_num_trackers_trackbar);
	on_num_trackers_trackbar(_feature_slider, 0);
	
	//trackbar for hue value
	string num_hue_title = "Hue";
	_hue_slider = 140;
	createTrackbar(num_hue_title, "Webcam", &_hue_slider, _hue_max, on_num_hue_trackbar);
	on_num_hue_trackbar(_hue_slider, 0);
	
	//trackbar for saturation value
	string num_saturation_title = "Saturation";
	_saturation_slider = 255;
	createTrackbar(num_saturation_title, "Webcam", &_saturation_slider, _saturation_max, on_num_saturation_trackbar);
	on_num_saturation_trackbar(_saturation_slider, 0);
	
	//trackbar for value value
	string num_value_title = "Value";
	_value_slider = 255;
	createTrackbar(num_value_title, "Webcam", &_value_slider, _value_max, on_num_value_trackbar);
	on_num_value_trackbar(_value_slider, 0);
	

	//trackbar for number of frames

	
	//points on prev frame and current frame
	vector<Point2f> points[2];

	//Read first frame
	camera >> old_frame;
	//cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	int key_pressed = 49;
	int mode = 49;

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
		inRange(hsv, Scalar(100, 150, 0), Scalar(_hue_slider, _saturation_slider, _value_slider), bin);
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

		
		//imshow("bin", bin);
		//Wait for a key to be pressed
		key_pressed = waitKey(5);
		mode = ((key_pressed < 49) || (key_pressed > 55)) ? mode : key_pressed;
		
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
			imshow("Webcam", bin);
		}

		//if user presses 7, then program ends
		if (key_pressed == 55) break;
	}

	//Success. The program accomplished its mission and now it can go
	// to the heaven of programs.
	return 0;
}