#pragma once
#include "Header.h"

class Detection
{
public:
	Detection();
	~Detection();
	int match(cv::Mat);
private:
	vector<Mat> object_images, descriptors_object;	
	vector<vector<KeyPoint>> keypoints_object;
	void Find_Features();
};

