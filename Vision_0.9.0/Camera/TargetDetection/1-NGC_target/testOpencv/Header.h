#include <opencv/cv.h>
#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <vector>
#include "FlyCapture2.h"
#include <math.h> 
#include <tchar.h>
#include <stdio.h>
#include <winsock2.h>
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define camera_angle 0.523599 //in rads
#define PI 3.14159265
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#define SERVER "127.0.0.1"  //ip address of udp server
#define BUFLEN 512  //Max length of buffer
#define PORT 8008   //The port on which to listen for incoming data

using namespace cv;
using namespace FlyCapture2;
using namespace std;

//in detection 
int detect_object(Mat frame, double *, double *);
void calibrate_HSV_range(Mat HSV);
void my_mouse_callback(int event, int x, int y, int flags, void* param);
void draw_box(IplImage* img, CvRect rect);
void get_color_range(Mat image);

//in source
void PrintBuildInfo();
void PrintCameraInfo(CameraInfo* pCamInfo);
void PrintError(Error error);
int RunCamera(PGRGuid guid);

