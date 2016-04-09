// LeahDLL.cpp : Defines the exported functions for the DLL application.

#include "LeahDLL.h"
#include <queue>

using namespace LeahDLL;
using namespace FlyCapture2;

#define PI 3.14159265

void Leah::initVideo(int VideoSource, int ImageWidth, int ImageHeight, int Threshold, bool Debug) {
/* Initializes several important members: 
 *      - capture, a pointer to the VideoCapture object
 *      - RefHist, a color histogram based on one or more reference images
 *      - _Targets, an intially empty list of Target objects
 * This method must be called before the camera can be used. 
 **************************************************************/

    ofstream logfile;
    logfile.open("visionLog.txt");
    logfile << "Vision video initialized" << endl;
    logfile.close();
    try {
        // Make connection to camera
        FlyCapture2::Camera camera;
        FlyCapture2::Error error;
		FlyCapture2::CameraInfo camInfo;

        error = camera.Connect(0);
        
        // Test if camera connection was successful
		if (error != FlyCapture2::PGRERROR_OK)
        {
            cout << "Failed to connect to camera" << endl;
			throw "Failed to connect camera.";
        }
        else {
            cout << "Camera connected successfully." << endl;
		}

		error = camera.GetCameraInfo(&camInfo);
		if (error != FlyCapture2::PGRERROR_OK){
			cout << "Failed to get camera info from camera" << endl;
			throw "Failed to get camera info from camera";
		}
		cout << camInfo.vendorName << " "
			 << camInfo.modelName << " "
			 << camInfo.serialNumber << endl;

		error = camera.StartCapture();
		if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED){
			cout << "Bandwidth exceeded" << endl;
			throw "Bandwidth exceeded";
		} 
		else if(error != PGRERROR_OK){
			cout << "Failed to start image capture" << endl;
			throw "Failed to start image capture";
		}

        capture = &camera;

        // Initialize the color histogram from one or more reference images
        initHistRef();

        _Targets = new list<Target>();
    }
    catch (const char* s) {
        cout << s << endl;
        cout << "Unable to continue, press any key to continue..." << endl;
        cin.get();
        exit(EXIT_FAILURE);
    }
}


void Leah::getFrame(Mat frame){
/* Gets a frame from the FlyCapture2 object, camera, and converts it
 * to an OpenCV Mat which is stored in the argument, frame.
 ********************************************************************/
    FlyCapture2::Image rawImage;
    
    // Get the image
    FlyCapture2::Error error = capture->RetrieveBuffer(&rawImage);
    
    // Check if getting the image was successful
    if (error != FlyCapture2::PGRERROR_OK)
    {
        cout << "Capture error" << endl;
	// We might want to throw an exception here?
    }

    // Convert to RGB
    FlyCapture2::Image rgbImage;
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

    // Convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
    frame = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
}

void Leah::initHistRef() {
/* Initializes the color histogram, RefHist, based on the frequency of
 * colors in one or more reference images. When searching for a target, we
 * will look for areas in the frame with colors that have high values 
 * in RefHist.
 ********************************************************************/
    int count = 2;
    Mat* referances = new Mat[count];
    for(int k = 0; k < count; k++) {
        string refpath = "referance";
        refpath += std::to_string(k);
        refpath += ".png";
        
        // Load histogram reference image
        Mat referance = imread(refpath);

        // Convert from Blue, Green, Red, to Hue, Saturation, Value for more accurate color detection
        // Place the result in the referances array
        cvtColor(referance,referances[k], CV_BGR2HSV);            
    }

    int h_bins = 30; int s_bins = 32;
    MatND* hist =new MatND(h_bins,s_bins,CV_8UC1);
    int histSize[] = { h_bins, s_bins };
    float h_range[] = { 0, 179 };
    float s_range[] = { 0, 255 };
    const float* ranges[] = { h_range, s_range };
    int channels[] = { 0, 1 };
    
    // Calculates the color histogram based on frequency of colors in reference images
    calcHist( referances, count, channels, Mat(), *hist, 2, histSize, ranges, true, false );
    
    // Linearly maps values in the histogram to the range [0, 255]
    normalize( *hist, *hist, 0, 255, NORM_MINMAX, -1, Mat() );

    // Sets values under 75 to 0 and values over 75 to 255
    threshold(*hist, *hist, 75, 255, THRESH_BINARY);

	//Set the internal reference histogram to the one we calculated
    RefHist = hist;
}


void Leah::process() {//Main processing function
    // Camera frame container
    Mat frame, mask;

    float h_range[] = { 0, 179 };
    float s_range[] = { 0, 255 };
    const float* ranges[] = { h_range, s_range };
    int channels[] = {0, 1};

    vector<vector<Point> > contours, contoursTwo;
    vector<Vec4i> hierarchy;
    int64 start, stop;
    double time;
    start = getTickCount();
    Mat target, passed;

    // Capture a frame from the camera and place it in frame
    getFrame(frame);

    // If there is a known target
    if(_Targets->size() > 0)
    {
        #pragma region TrackingCode
        // Create bounding rectangle around the target, with some added leeway
        Rect bounds = Rect(_Targets->front().CenterX - _Targets->front().Radius * 1.25, 
                           _Targets->front().CenterY - _Targets->front().Radius * 1.25,  
                           _Targets->front().Radius * 2.5, _Targets->front().Radius * 2.5);
        
        // bounds.x and bounds.y are the coordinates of the top left corner.
        
        // Truncate the rectangle if it goes outside the frame
        if(bounds.x < 0) {
			bounds.width += bounds.x;
            bounds.x = 0;
        }
        if(bounds.x + bounds.width > frame.cols) {
            bounds.width = frame.cols - bounds.x ;
        }

        if (bounds.width < _Targets->front().Radius) {          
            goto exit;
        }

        // Truncate the rectangle if it goes outside the frame
        if(bounds.y < 0) {
			bounds.height += bounds.y;
            bounds.y = 0;
        }
        if(bounds.y + bounds.height > frame.rows) {
            bounds.height = frame.rows - bounds.y;
        }    
        
        if(bounds.height < _Targets->front().Radius)
        {
            goto exit;
        }

        // Take a slice of frame with coordinates given by the rectangle
        Mat tempInput;
        frame(bounds).copyTo(tempInput);

        // Check if the slice has nonpositive dimensions
        if(tempInput.rows <= 0 || tempInput.cols <= 0) {
            goto exit;
        }

        // Creates the result matrix of the required dimensions.
        // See OpenCV2 documentation for matchTemplate.
        int result_cols = tempInput.cols - _Targets->front().image.cols + 1;
        int result_rows = tempInput.rows - _Targets->front().image.rows + 1;
        Mat result;
        result.create( result_cols, result_rows, CV_32FC1 );
                
        // Populates the result matrix by comparing tempInput to the previous
        // image of the target. Higher values mean a closer match.
        matchTemplate( tempInput, _Targets->front().image, result, 5);

        // Linearly maps the values in result to the range [0, 1]
        normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

        // Localizing the best match with minMaxLoc
        double minVal; double maxVal; Point minLoc; Point maxLoc;
        Point matchLoc;

        minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

        matchLoc = maxLoc; 

        // Check if result contains a close match
        if(maxVal > 0.85) {
            _Targets = new list<Target>();
            int dilation_size = 5;
            Mat element = getStructuringElement( MORPH_RECT,
                                                 Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                 Point( dilation_size, dilation_size ) );
            Mat passTwo,maskTwo;//,sourceTwo;
            vector<Vec3f> circles;
            vector<Vec4i> hierarchyTwo;
            //Rect test = Rect(boxes[i].tl() * 4,boxes[i].br() *4);
            //frame(test).copyTo(sourceTwo);
                    
            //Convert from Blue, Green, Red, to Hue, Saturation, Value for more accurate color detection
            cvtColor(tempInput,maskTwo,CV_BGR2HSV);

            blur(maskTwo,maskTwo,Size(5,5));
            calcBackProject( &maskTwo, 1, channels, *RefHist, passTwo, ranges,4, true );
            threshold(passTwo,passTwo,70 ,255,THRESH_BINARY);
            dilate(passTwo,passTwo,element);
            //imshow("passTwo",passTwo);
            findContours( passTwo, contoursTwo, hierarchyTwo, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            vector<vector<Point> > contours_poly( contoursTwo.size() );
            vector<Rect> boxesTwo( contoursTwo.size() );
            vector<Point2f>center( contoursTwo.size() );
            vector<float>radius( contoursTwo.size() );
            vector<double>area (contoursTwo.size() );
            float largest = 0;
            //cout << "Number of shapes: "<< contoursTwo.size() <<'\n';
            for( int j = 0; j < contoursTwo.size(); j++ ) { 
                approxPolyDP( Mat(contoursTwo[j]), contours_poly[j], 3, true );
                boxesTwo[j] = boundingRect( Mat(contours_poly[j]) );
                area[j]  = boxesTwo[j].width * boxesTwo[j].height;//= contourArea(contoursTwo[j]);
                if(area[j] > largest)
                    largest = area[j];
                        
                minEnclosingCircle( (Mat)contours_poly[j], center[j], radius[j] );
            }

            for( int j = 0; j < center.size(); j++ ) {
                if(area[j] >= largest * 0.80) {
                    rectangle(tempInput,boxesTwo[j].tl() ,boxesTwo[j].br() ,Scalar(0,255,0),2,8,0);
                    //cout << "Circle test \n"; 
                    float cArea = (3.14 * radius[j] * radius[j]);
                    if( radius[j] > 0) {//&& (area[j] * 0.785 < cArea + cArea*0.10 ))//0.40 /*boxesTwo[j].area()*/
                        Target tar;
                        //cout << "Circle found \n";
                                
                        circle(tempInput, Point(center[j]) , (int)radius[j] , Scalar(255,0,0), 2, 8, 0 );
                        putText(frame,"Target",Point(center[j])  + bounds.tl(),0,1,Scalar(255,0,0),2,8,false);
                        circle(frame,Point(center[j]) + bounds.tl(),radius[j],Scalar(0,255,0));
                        Rect tarbounds = Rect(bounds.tl().x + center[j].x - radius[j],bounds.tl().y + center[j].y - radius[j],radius[j],radius[j]);
                        if(tarbounds.tl().x < 0)
                        {
                            tarbounds.x =0;
                        }
                        if(tarbounds.tl().y < 0)
                        {
                            tarbounds.y =0;
                        }
                        if(tarbounds.tl().x + tarbounds.width > frame.cols)
                        {
                            tarbounds.width = frame.cols - tarbounds.tl().x ;
                            if(tarbounds.width < radius[j]*2)
                            {
                                continue;
                            }
                        }
                        if(tarbounds.tl().y + tarbounds.height > frame.rows)
                        {
                            tarbounds.height = frame.rows - tarbounds.tl().y;
                            if(tarbounds.height <  radius[j]*2)
                            {
                                continue;
                            }
                        }
                        frame(tarbounds).copyTo(tar.image);
                        tar.CenterX = center[j].x + bounds.tl().x;
                        tar.CenterY = center[j].y + bounds.tl().y;
                        tar.Radius = radius[j];        
                        tar.Heading = atan2(tar.CenterX - ImgWidth/2, ImgWidth / (2 * tan(FOV/2)));
                        _Targets->push_front(tar);

                    }
                }
            }
            if(DEBUG)
            {
                stop = getTickCount();
                time = (stop - start)/getTickFrequency();
                int chan = frame.channels();
                int col = frame.cols * chan;
                putText(frame,std::to_string(1/time),Point(100,200),0,1,Scalar(255,255,255),2,8,false);
                imshow("Input",frame);
                waitKey(5);
            }
            return;
        }
        //putText(tempInput,std::to_string(maxVal),Point(25,25),1,1,Scalar(0,255,0));
        /// Show me what you got
        //rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
        //rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
        //imshow("temp in",tempInput);
        //imshow("Match", result );
        //imshow( result_window, result );
        #pragma endregion TrackingCode
    }
    exit:
    //Scale down the source image by a factor of 4
    //This is done to reduce noise and increase framerate
    pyrDown(frame, mask, Size(frame.cols/2, frame.rows/2));
    pyrDown(mask, mask, Size(mask.cols/2, mask.rows/2));

    //Convert from Blue, Green, Red, to Hue, Saturation, Value for more accurate color detection
    cvtColor(mask,mask,CV_BGR2HSV);

    //normalize(mask,mask,0,255,
    calcBackProject( &mask, 1, channels, *RefHist, passed, ranges, 2, true );
    blur(passed,passed,Size(5,5));
    //imshow("stage 1", passed);
    threshold(passed,passed,THRESHOLD ,255,THRESH_BINARY);
    Canny(passed,passed,30,90);
    int dilation_size = 5;
    Mat element = getStructuringElement( MORPH_RECT,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );
    dilate(passed,passed,element);
    findContours( passed, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boxes( contours.size() );
    for( int j = 0; j < contours.size(); j++ )
    { 
        approxPolyDP( Mat(contours[j]), contours_poly[j], 3, true );
        boxes[j] = boundingRect( Mat(contours_poly[j]) );
        boxes[j].height = boxes[j].width;
        rectangle(frame,boxes[j].tl() * 4,boxes[j].br() * 4,Scalar(0,0,255),2,8,0);

    }
    _Targets = new list<Target>();
    //Test if we can do a second pass
    for(int i = 0; i < boxes.size();i++)
    {
        if(boxes[i].area() > 100)
        {
            Mat passTwo,maskTwo,sourceTwo;
            vector<Vec3f> circles;
            vector<Vec4i> hierarchyTwo;
            Rect test = Rect(boxes[i].tl() * 4,boxes[i].br() *4);

            if(test.tl().x < 0)
                test.x = 0;
            if(test.tl().y < 0)
                test.y = 0;
            if(test.width + test.x > frame.cols)
            {
                test.width = frame.cols - test.x;
            }
            if(test.height + test.y > frame.rows)
            {
                test.height = frame.rows - test.y;
            }
            if(test.width <= 0)
                continue;
            if(test.height <= 0)
                continue;

            frame(test).copyTo(sourceTwo);
            cvtColor(sourceTwo,maskTwo,CV_BGR2HSV);//Color space conversion for more accurate color detection
            blur(maskTwo,maskTwo,Size(5,5));
            calcBackProject( &maskTwo, 1, channels, *RefHist, passTwo, ranges,2, true );//was 4
            threshold(passTwo,passTwo,THRESHOLD ,255,THRESH_BINARY);
            dilate(passTwo,passTwo,element);
            //imshow("passTwo",passTwo);
            findContours( passTwo, contoursTwo, hierarchyTwo, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            vector<vector<Point> > contours_poly( contoursTwo.size() );
            vector<Rect> boxesTwo( contoursTwo.size() );
            vector<Point2f>center( contoursTwo.size() );
            vector<float>radius( contoursTwo.size() );
            vector<double>area (contoursTwo.size() );
            float largest = 0;
            //cout << "Number of shapes: "<< contoursTwo.size() <<'\n';
            for( int j = 0; j < contoursTwo.size(); j++ )
            { 
                //cout << "Pass " << j << '\n';
                approxPolyDP( Mat(contoursTwo[j]), contours_poly[j], 3, true );
                boxesTwo[j] = boundingRect( Mat(contours_poly[j]) );
                area[j]  = boxesTwo[j].width * boxesTwo[j].height;//= contourArea(contoursTwo[j]);
                if(area[j] > largest)
                    largest = area[j];
                        
                minEnclosingCircle( (Mat)contours_poly[j], center[j], radius[j] );
                            
            }
            for( int j = 0; j < center.size(); j++ )
            {
                if(area[j] >= largest * 0.80)
                {
                    rectangle(sourceTwo,boxesTwo[j].tl() ,boxesTwo[j].br() ,Scalar(0,255,0),2,8,0);
                    //cout << "Circle test \n"; 
                    float cArea = (3.14 * radius[j] * radius[j]);
                    if((area[j] * 0.785 > cArea - cArea*0.50 || area[j] * 0.785 < cArea + cArea*0.50 )  && radius[j] > 0)//&& (area[j] * 0.785 < cArea + cArea*0.10 ))//0.40 /*boxesTwo[j].area()*/
                    {
                        Target tar;
                        circle( sourceTwo, Point(center[j]) , (int)radius[j] , Scalar(255,0,0), 2, 8, 0 );
                        putText(frame,"Target",Point(center[j])  + boxes[i].tl() * 4,0,1,Scalar(255,0,0),2,8,false);
                        tar.CenterX = center[j].x + boxes[i].tl().x * 4;
                        tar.CenterY = center[j].y + boxes[i].tl().y * 4;
                        if(tar.CenterX < 0 || tar.CenterX > frame.cols)
                        {
                            continue;
                        }
                        if(tar.CenterY < 0 || tar.CenterY > frame.rows)
                        {
                            continue;
                        }
                        Rect bounds = Rect(boxes[i].tl().x * 4 + center[j].x - radius[j],
                                           boxes[i].tl().y * 4 + center[j].y - radius[j],
                                           radius[j] ,radius[j] );
                                
                        if(bounds.tl().x < 0)
                        {
                            bounds.x =0;
                        }
                        if(bounds.tl().y < 0)
                        {
                            bounds.y =0;
                        }
                        if(bounds.tl().x + bounds.width > frame.cols)
                        {
                            bounds.width = frame.cols - bounds.tl().x ;
                            if(bounds.width < radius[j])
                            {
                                continue;
                            }
                        }
                        if(bounds.tl().y + bounds.height > frame.rows)
                        {
                            bounds.height = frame.rows - bounds.tl().y;
                            if(bounds.height <  radius[j])
                            {
                                continue;
                            }
                        }
                        frame(bounds).copyTo(tar.image);
                                
                        tar.Radius = radius[j];
                                
                        tar.Heading = atan2(tar.CenterX - ImgWidth/2, ImgWidth / (2 * tan(FOV/2)));

                        _Targets->push_front(tar);
                    }
                }
            }
            //imshow("maskTwo",sourceTwo);
        }
    }
    if(DEBUG)
    {
        stop = getTickCount();
        time = (stop - start)/getTickFrequency();
        int chan = frame.channels();
        int col = frame.cols * chan;
        putText(frame,std::to_string(1/time),Point(100,200),0,1,Scalar(255,255,255),2,8,false);
        imshow("Input",frame);
        waitKey(5);
    }
}

float Leah::mapHeading(int u, int v) {
/* Maps a location in frame to an angle in radians, relative to the
 * current heading, i.e. negative means target is to the left, positive
 * to the right, zero dead ahead.
 *************************************************/
    float x = (float)(u - ImgWidth/2);
    float y = (float)(ImgWidth / (2 * tan(FOV/2)));
    return (float)atan2(x, y);  
}

bool Leah::getTarget(int* data, int size)
{
    int used = 0;
    if(size < _Targets->size() * 3){
        return false;
    }

    list<Target> working = list<Target>(*_Targets);
    while(working.size() > 0)
    {
        (*data++) = working.front().CenterX;
        (*data++) = working.front().CenterY;
        (*data++) = working.front().Radius;
        working.pop_front();
    }
    return true;
}

bool Leah::targetFound() {
    return (_Targets->size() > 0 ); 
}

void Leah::initLIDAR(bool Debug,float scale,int maxrange)
{
    DEBUG = Debug;
    SCALE = scale;
    MaxRange = maxrange;
}