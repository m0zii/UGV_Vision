#ifndef LEAHDLL_H
#define LEAHDLL_H

#pragma once

#include <winSock2.h>
#include <windows.h>
#include <iostream>
#include <stdlib.h>
#include <climits>
#include <cerrno>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <math.h>
#include <string.h>
#include <fstream>

#include "FlyCapture2.h"

#define PI 3.14159265359
using namespace std;
using namespace cv;

struct Target{
public: 
    int CenterX;
    int CenterY;
    int tlx;
    int tly;
    int brx;
    int bry;
    float angle;
    float Heading;
    int Radius;
    bool valid;
    int FramesSinceLast;
    int ID;
    Mat image;
};

namespace LeahDLL{
    public ref class Leah
    {
            
    public:
        Leah(){
            ImgWidth = 2048;
            ImgHeight = 2048;
            DEBUG = 1;
            THRESHOLD = 50;
            FOV = PI/2;
        };
        virtual void initVideo(int VideoSource,int ImgWidth,int ImgHeight,int Threshold,bool Debug);
        virtual void initLIDAR(bool Debug,float scale,int maxrange);
        virtual void process();
        virtual bool targetFound();
        virtual bool getTarget(int* data, int size);
        virtual void Mapping(int* data,int size, int angle, float bearing,int x,int y);
		virtual float mapHeading(int x, int y);
    private:
        void initHistRef();
        static list<Target>* _Targets;
        static FlyCapture2::Camera* capture;
        static Mat* Vidframe;
        static int ImgWidth;
        static int ImgHeight;
        static bool DEBUG ;
        static int THRESHOLD ;
        static double FOV;
        static MatND* RefHist;
        static float SCALE;
        static int MaxRange;
		void getFrame(Mat frame);
    };
}

#endif