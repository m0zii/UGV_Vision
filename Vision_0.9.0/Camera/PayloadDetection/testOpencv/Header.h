#include <opencv/cv.h>
#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include <vector>
#include "FlyCapture2.h"
#include <math.h> 
#include "Detection.h"
#include <Windows.h>

#define camera_angle 0.523599 //in rads
#define PI 3.14159265
#define NUMBER_IMG 1
#define LOAD_FROM "C:/Users/MadMinute/Desktop/NGC/"

using namespace cv;
using namespace FlyCapture2;
using namespace std;

//in source
void PrintBuildInfo();
void PrintCameraInfo(CameraInfo* pCamInfo);
void PrintError(Error error);
int RunCamera(PGRGuid guid);

