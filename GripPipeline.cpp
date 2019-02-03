#include "GripPipeline.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
/**
* Initializes a GripPipeline.
*/
using namespace frc;
GripPipeline::GripPipeline() {
	SmartDashboard::PutNumber("Hue Min", 60);
	SmartDashboard::PutNumber("Hue Max", 90);
	SmartDashboard::PutNumber("Sat Min", 70);
	SmartDashboard::PutNumber("Sat Max", 255);
	SmartDashboard::PutNumber("Val Min", 130);
	SmartDashboard::PutNumber("Val Max", 255);
}
/**
* Runs an iteration of the Pipeline and updates outputs.
*
* Sources need to be set before calling this method. 
*
*/
void GripPipeline::Process(cv::Mat &source){
	// Resize Image
	cv::Mat resizeImageInput = source;
	double resizeImageWidth = 320.0;
	double resizeImageHeight = 240.0;
	int resizeImageInterpolation = cv::INTER_CUBIC;
	resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, this->resizeImageOutput);
	// Find Image Part That matches HSV Threshold
	cv::Mat hsvThresholdInput = resizeImageOutput;
	double hsvThresholdHue[] = {SmartDashboard::GetNumber("Hue Min", 60), SmartDashboard::GetNumber("Hue Max",90)};
	double hsvThresholdSaturation[] = {SmartDashboard::GetNumber("Sat Min", 70), SmartDashboard::GetNumber("Sat Max", 255)};
	double hsvThresholdValue[] = {SmartDashboard::GetNumber("Val Min", 130), SmartDashboard::GetNumber("Val Max", 255)};
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
	// Remove part of image that matches HSV Threshold
	cv::Mat cvErodeSrc = hsvThresholdOutput;
	cv::Mat cvErodeKernel;
	cv::Point cvErodeAnchor(-1, -1);
	double cvErodeIterations = 1.0;
    int cvErodeBordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvErodeBordervalue(-1);
	cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, this->cvErodeOutput);
	// Find Contours on eroded image
	cv::Mat findContoursInput = cvErodeOutput;
	bool findContoursExternalOnly = false;
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);

	// Filter Contours by Height to Width Ratio
	filterContoursOutput.clear();
	contour_pairs.clear();
	rectangles.clear();
	double min_rect_ratio = 1.0;
	double max_rect_ratio = 4.0;
	for (auto &cont : findContoursOutput){
		cv::Rect temp = cv::boundingRect(cv::Mat(cont));
		double ratio = temp.height/static_cast<double>(temp.width);
		if (ratio > min_rect_ratio && ratio < max_rect_ratio){
			filterContoursOutput.push_back(cont);
			rectangles.push_back(temp);			
		}
	}

	// Finds pairs of rectangles that are facing each other & have relatively equal area
	double min_area_ratio = 0.5;
	double max_area_ratio = 2.0;
	if (rectangles.size()>1 && filterContoursOutput.size()>1) {
		for (int i = 0; i<rectangles.size()-1; i++){
			for (int j = i+1; j<rectangles.size(); j++) {
				double rect_ratio = static_cast<double>(rectangles[i].area())/rectangles[j].area();

				// Makes sure the areas are relatively equal
				if (rect_ratio > min_area_ratio && rect_ratio < max_area_ratio){
					//std::cout<<"Ratio valid: " << rect_ratio <<std::endl;

					cv::Rect rect1;
					cv::Rect rect2;
					std::vector<cv::Point> contour1;
					std::vector<cv::Point> contour2;

					// Make it so rect1 is always on the left and rect2 is on the right
					if (rectangles[i].x > rectangles[j].x){ //j is to the left, so it's rect1
						rect1 = rectangles[j];
						contour1 = filterContoursOutput[j];
						rect2 = rectangles[i];
						contour2 = filterContoursOutput[i];
					} else { //j is to the right, so it's rect2
						rect1 = rectangles[i];
						contour1 = filterContoursOutput[i];
						rect2 = rectangles[j];
						contour2 = filterContoursOutput[j];
					}

					cv::Point farPoint1 = findFarthestPoint(contour1, RIGHT);
					cv::line(resizeImageOutput, farPoint1, farPoint1, {255,0,0},4);
					cv::Point farPoint2 = findFarthestPoint(contour2, LEFT);
					cv::line(resizeImageOutput, farPoint2, farPoint2, {255,0,0},4);

					int rect1MidY = (rect1.tl().y+rect1.br().y)/2;
					int rect2MidY = (rect2.tl().y+rect2.br().y)/2;

					// Check that the rectangles are facing each other
					if (farPoint1.y < rect1MidY && farPoint2.y < rect2MidY){
						cv::rectangle(resizeImageOutput, rect1, {0,255,255}, 2);
						cv::rectangle(resizeImageOutput, rect2, {0,255,255}, 2);
						int midX = (rect1.tl().x+rect2.br().x)/2;
						cv::line(resizeImageOutput, cv::Point{midX,0}, cv::Point{midX,240}, {0,0,255},5);
						SmartDashboard::PutNumber("Vision Mid X", midX);
						//std::cout<<"Valid orientation"<<std::endl;
					} else {
						//std::cout<<"Invalid orientation"<<std::endl;
					}
				} else {
					//std::cout<<"Bad ratio: " << rect_ratio <<std::endl;
				}
				//std::cout<<std::endl;
			}
		}
	}
}
	
// Checks if the area of A > B
bool GripPipeline::compareRectAreas(cv::Rect a, cv::Rect b){
	return static_cast<double>(a.area()) > static_cast<double>(b.area());
}
// Finds the furthest point in the specified direction with respect to the x coordinate
cv::Point GripPipeline::findFarthestPoint(std::vector<cv::Point> contour, Direction direction){ //contour used to be a reference
	
	cv::Point temp_point = contour[0];
	
	//for (cv::Point test_point : contour){
	for (auto test_point : contour) {
		if ((direction==RIGHT && test_point.x > temp_point.x) || (direction==LEFT && test_point.x < temp_point.x)){
			temp_point = test_point;
		}
	}
	return temp_point; 	
}

/**
 * Scales and image to an exact size.
 *
 * @param input The image on which to perform the Resize.
 * @param width The width of the output in pixels.
 * @param height The height of the output in pixels.
 * @param interpolation The type of interpolation.
 * @param output The image in which to store the output.
 */
void GripPipeline::resizeImage(cv::Mat &input, double width, double height, int interpolation, cv::Mat &output) {
	cv::resize(input, output, cv::Size(width, height), 0.0, 0.0, interpolation);
}

/**
 * Segment an image based on hue, saturation, and value ranges.
 *
 * @param input The image on which to perform the HSL threshold.
 * @param hue The min and max hue.
 * @param sat The min and max saturation.
 * @param val The min and max value.
 * @param output The image in which to store the output.
 */
void GripPipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
	cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
	cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
}

/**
 * Expands area of lower value in an image.
 * @param src the Image to erode.
 * @param kernel the kernel for erosion.
 * @param anchor the center of the kernel.
 * @param iterations the number of times to perform the erosion.
 * @param borderType pixel extrapolation method.
 * @param borderValue value to be used for a constant border.
 * @param dst Output Image.
 */
void GripPipeline::cvErode(cv::Mat &src, cv::Mat &kernel, cv::Point &anchor, double iterations, int borderType, cv::Scalar &borderValue, cv::Mat &dst) {
	cv::erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
}

/**
 * Finds contours in an image.
 *
 * @param input The image to find contours in.
 * @param externalOnly if only external contours are to be found.
 * @param contours vector of contours to put contours in.
 */
void GripPipeline::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
	std::vector<cv::Vec4i> hierarchy;
	contours.clear();
	int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
	int method = cv::CHAIN_APPROX_SIMPLE;
	cv::findContours(input, contours, hierarchy, mode, method);
}

