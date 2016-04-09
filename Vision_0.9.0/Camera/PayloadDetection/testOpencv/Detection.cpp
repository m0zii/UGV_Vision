#include "Detection.h"

Detection::Detection()
{
	for (int i = 0; i < NUMBER_IMG; i++)
	{
		//load object
		Mat object_img = imread(LOAD_FROM + to_string((long long)i) + ".jpg", CV_LOAD_IMAGE_GRAYSCALE);
		if (!object_img.data)
		{
			throw exception("Images failed to load correctly");
		}
		object_images.push_back(object_img);
	}
	Find_Features();
}
void Detection::Find_Features(){
	//Detect the keypoints using SURF Detector
	int minHessian = 400;
	//int minHessian = 200;
	SurfFeatureDetector detector(minHessian);

	//Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;

	for (int i = 0; i < NUMBER_IMG; i++)
	{
		vector<KeyPoint> temp_keypoints;
		Mat temp_decripter;
		object_images.at(i);
		detector.detect(object_images.at(i), temp_keypoints);
		keypoints_object.push_back(temp_keypoints);
		extractor.compute(object_images.at(i), keypoints_object.at(i), temp_decripter);
		descriptors_object.push_back(temp_decripter);
	}
}

int Detection::match(Mat image){
	Mat search_img;
	cvtColor(image, search_img, CV_BGR2GRAY);
	//Detect the keypoints using SURF Detector
	int minHessian = 400;

	SurfFeatureDetector detector(minHessian);
	vector<KeyPoint> keypoints_scene;

	detector.detect(search_img, keypoints_scene);

	//Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;

	Mat descriptors_scene;

	extractor.compute(search_img, keypoints_scene, descriptors_scene);
	if (descriptors_scene.empty()){
		cout << "Image skipped, no discriptors found for image" << endl;
		return 0;
	}
	//Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	vector< vector< DMatch >  > matches;
	for (int i = 0; i < NUMBER_IMG; i++)
	{
		matcher.knnMatch(descriptors_object.at(i), descriptors_scene, matches, 2);
		//matcher.Match(descriptors_object, descriptors_scene, matches);

		double max_dist = 0; double min_dist = 50;

		vector< DMatch > good_matches;
		good_matches.reserve(matches.size());

		for (int j = 0; j < matches.size(); j++)
		{
			if (matches[j].size() < 2)
				continue;

			const DMatch &m1 = matches[j][0];
			const DMatch &m2 = matches[j][1];

			if (m1.distance <= .45 * m2.distance)
				good_matches.push_back(m1);
		}
		Mat img_matches, final_image = image;
		//check if too few good matches
		if (good_matches.size() < 4){
			imshow("Object detection", final_image);
			int userinput = waitKey(1);
			if (userinput == 's')
			{
				string image_name = "";
				cout << "Please enter the name of the image: ";
				cin >> image_name;
				cout << endl;
				imwrite("C:/Users/MadMinute/Desktop/NGC/" + image_name + ".jpg", image);
			}
			else if (userinput == 'q')
				return 1;
			else
				return 0;
		}
		//Localize the object
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for (int j = 0; j < good_matches.size(); j++)
		{
			//Get the keypoints from the good matches
			obj.push_back(keypoints_object.at(i)[good_matches[j].queryIdx].pt);
			scene.push_back(keypoints_scene[good_matches[j].trainIdx].pt);
		}

		Mat H = findHomography(obj, scene, CV_RANSAC);
		//Get the corners from the image_1 ( the object to be "detected" )
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cvPoint(0, 0);
		obj_corners[1] = cvPoint(object_images.at(i).cols, 0);
		obj_corners[2] = cvPoint(object_images.at(i).cols, object_images.at(i).rows);
		obj_corners[3] = cvPoint(0, object_images.at(i).rows);
		std::vector<Point2f> scene_corners(4);

		perspectiveTransform(obj_corners, scene_corners, H);

		//check if points make sense.
		/*double slope[4], intercept[4];
		for (int i = 0; i < 4; i++)
		{
			double dx, dy, x1 = scene_corners[i].x, y1 = scene_corners[i].y, x2 = scene_corners[i+1].x, y2 = scene_corners[i+1].y;
			dx = x2 - x1;
			dy = y2 - y1;
			slope_line1 = dy / dx;
			intercept_line1 = y1 - slope_line1 * x1;
		}*/



		//Draw lines between the corners (the mapped object in the scene - image_2 )
		line(final_image, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 4);
		line(final_image, scene_corners[1], scene_corners[2], Scalar(0, 255, 0), 4);
		line(final_image, scene_corners[2], scene_corners[3], Scalar(0, 255, 0), 4);
		line(final_image, scene_corners[3], scene_corners[0], Scalar(0, 255, 0), 4);

		//Show detected matches
		imshow("Object detection", final_image);
		int userinput = waitKey(1);
		if (userinput == 's')
		{
			string image_name = "";
			cout << "Please enter the name of the image: ";
			cin >> image_name;
			cout << endl;
			imwrite("C:/Users/MadMinute/Desktop/NGC/" + image_name + ".jpg", image);
		}
		else if (userinput == 'q')
			return 1;
	}
}
Detection::~Detection()
{
}
