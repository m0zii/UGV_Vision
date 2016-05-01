#include "Header.h"

bool destroy = false;
CvRect box;
bool drawing_box = false;
//covers red on low end of color space
int lowH_low = 0;
int highH_low = 10;
int lowS = 70;
int highS = 200;
int lowV = 50;
int highV = 200;
//covers red on high end of color space
int lowH_high = 170;
int highH_high = 180;

int detect_object(Mat scene, double *distance_f, double *angle_f){

	Mat frame,HSV, img_thresholded;

	//convert to HSV color space
	cvtColor(scene, HSV, CV_BGR2HSV);

	//On HSV color wheel red is split by 0. the hue for red can be in the high 170s and low 10s
	//inrange will only let you select 1 range, therefore you need to use this funtion twice to capture
	//the two ranges that define red
	Mat mask1, mask2;
	//gets the color that falls in the range of the scalar
	inRange(HSV, Scalar(lowH_low, lowS, lowV), Scalar(highH_low, highS, highV), mask1);
	inRange(HSV, Scalar(lowH_high, lowS, lowV), Scalar(highH_high, highS, highV), mask2);
	
	//or the two ranges together
	img_thresholded= mask1 | mask2;

	//reduce noise by filling and dilating abnormal detections
	dilate(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(img_thresholded, img_thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	// gaussian image blur
	GaussianBlur(img_thresholded, img_thresholded, Size(9, 9), 0, 0);

	//detect circles
	vector<Vec3f> circles;
	HoughCircles(img_thresholded, circles, CV_HOUGH_GRADIENT,3, img_thresholded.rows / 2, 200, 100, 0, img_thresholded.rows);
	if (circles.size() < 1){
		//draw circles
		for (size_t i = 0; i < 1; i++)//circles.size(); i++)
		{
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			cout << "x=" << cvRound(circles[i][0]) << "y=" << cvRound(circles[i][1]) << "r=" << cvRound(circles[i][2]) << endl;
			int radius = cvRound(circles[i][2]);
			// circle center
			circle(scene, center, 3, Scalar(0, 255, 0), -1, 8, 0);
			// circle outline
			circle(scene, center, radius, Scalar(0, 0, 255), 3, 8, 0);

			//calc distance of circle from UGV
			//this should probably not go here
			double distance = (21.0 * 220.0) / circles[i][2];
			double midpoint = scene.cols / 2.0;
			double opposite = circles[i][0] - midpoint;
			double pixel_length = (2 * distance*tan(camera_angle / 2)) / scene.cols;
			double angle = atan2(distance, (opposite* pixel_length)) * 180 / 3.15;
			cout << "Distance= " << distance << " Feet" << endl;
			*distance_f = distance*.3048;
			cout << "Angle= " << angle << "degrees" << endl;
			if ((angle - 90) > 0){
				angle -= 90;
			}
			else{
				angle = (angle - 90) + 360;
			}
			*angle_f = angle*.0174533;
			if (i > 2)
				break;
		}
	}
	else{
		*distance_f = -1;
		*angle_f = -1;
	}
	//test
	//Mat scaled, scaled_thresh;
	//resize(scene, scaled, cvSize(scene.cols*.25, scene.rows*.25));
	//resize(img_thresholded, scaled_thresh, cvSize(img_thresholded.cols*.25, img_thresholded.rows*.25));
	//imshow("img_thresholded", scaled_thresh);
	//imshow("scene", scaled);
	
	//display image
	imshow("img_thresholded", img_thresholded);
	imshow("scene", scene);
	int input=waitKey(1);

	//if user inputs c the calibration is performed
	if (input == 'c')
		calibrate_HSV_range(HSV);

	//murder program in cold blood... no warning, just death
	if (input == 'q')
		return 1; //retunrs dont kill programs, users do
	return 0;
}

void calibrate_HSV_range(Mat HSV){
	IplImage img = HSV.operator IplImage();
	IplImage* image = &img;


	cvNamedWindow("calibrate");
	box = cvRect(0, 0, 1, 1);
	IplImage* temp = cvCloneImage(image);

	// Set up the callback
	cvSetMouseCallback("calibrate", my_mouse_callback, (void*)image);

	
	while (true)
	{
		if (destroy)
		{
			cvDestroyWindow("calibrate"); 
			break;
		}
		cvCopyImage(image, temp);

		//draw box on image
		if (drawing_box)
			draw_box(temp, box);

		cvMoveWindow("calibrate", 200, 100);
		cvShowImage("calibrate", temp);

		if (cvWaitKey(15) == 27)
			break;
	}

	//take care of memory
	cvReleaseImage(&temp);
	cvDestroyWindow("calibrate");

	// Retrieve a single frame from the device and set the ROI
	IplImage* vid_frame = &img;

	//set image region of interest
	cvSetImageROI(vid_frame, box);

	//reset values
	destroy = false;
	drawing_box = false;

	//get seperate RIO image 
	Mat RIO_img(vid_frame);
	//Get HSV color ranges
	get_color_range(RIO_img);
}
void get_color_range(Mat image){
	lowH_low = 180;
	highH_low = 0;
	lowS = 150;
	highS = 0;
	lowV = 150;
	highV = 0;
	lowH_high =180;
	highH_high = 0;

	//runs for each pixel of image
	for (int j = 0; j < image.rows; j++)
	{
		for (int k = 0; k < image.cols; k++)
		{
			//breaks pixel into H, S, V values
			Vec3b pixel = image.at<cv::Vec3b>(j, k);
			for (int i = 0; i < 3; i++)
			{
				//this splits the color values into low and high. red HSV can be in 2 ranges.
				switch (i) {
					case 0://H
						//low range
						if (pixel[i] < 90){
							if (pixel[i] > highH_low)
								highH_low = pixel[i];
							if (pixel[i] < lowH_low)
								lowH_low = pixel[i];
						}
						//high range
						else{
							if (pixel[i] > highH_high)
								highH_high = pixel[i];
							if (pixel[i] < lowH_high)
								lowH_high = pixel[i];
						}
						break;
					case 1://S
						if (pixel[i] > highS)
							highS = pixel[i];
						if (pixel[i] < lowS && pixel[i]>100)//100 is artificial limit to filter out white
							lowS = pixel[i];
						break;
					case 2://V
						if (pixel[i] > highV)
							highV = pixel[i];
						if (pixel[i] < lowV && pixel[i]>100)//100 is artificial limit to filter out black
							lowV = pixel[i];
						break;
					default:
						break;
				}
			}
		}
	}
}
	
void draw_box(IplImage* img, CvRect rect)
{
	cvRectangle(img, cvPoint(box.x, box.y), cvPoint(box.x + box.width, box.y + box.height),
		cvScalar(0, 0, 255), 2);

	CvRect rect2 = cvRect(box.x, box.y, box.width, box.height);
}

//mouse callback
void my_mouse_callback(int event, int x, int y, int flags, void* param)
{
	IplImage* frame = (IplImage*)param;

	switch (event)
	{
	case CV_EVENT_MOUSEMOVE:
	{
		if (drawing_box)
		{
			box.width = x - box.x;
			box.height = y - box.y;
		}
	}
		break;

	case CV_EVENT_LBUTTONDOWN:
	{
		drawing_box = true;
		box = cvRect(x, y, 0, 0);
	}
		break;

	case CV_EVENT_LBUTTONUP:
	{
		drawing_box = false;
		if (box.width < 0)
		{
			box.x += box.width;
			box.width *= -1;
		}

		if (box.height < 0)
		{
			box.y += box.height;
			box.height *= -1;
		}

		draw_box(frame, box);
	}
		break;

	case CV_EVENT_RBUTTONUP:
	{
		destroy = true;
	}
		break;

	default:
		break;
	}
}