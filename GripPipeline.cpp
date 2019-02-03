#include "GripPipeline.h"
#include <frc/smartdashboard/SmartDashboard.h>
/**
* Initializes a GripPipeline.
*/
using namespace frc;
GripPipeline::GripPipeline() {}
/**
* Runs an iteration of the Pipeline and updates outputs.
*
* Sources need to be set before calling this method. 
*
*/
void GripPipeline::Process(cv::Mat &source){
	//Step Resize_Image0:
	//input
	cv::Mat resizeImageInput = source;
	double resizeImageWidth = 320.0;  // default Double
	double resizeImageHeight = 240.0;  // default Double
	int resizeImageInterpolation = cv::INTER_CUBIC;
	resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, this->resizeImageOutput);
	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThresholdInput = resizeImageOutput;
	double hsvThresholdHue[] = {60, 90};
	double hsvThresholdSaturation[] = {30, 255};
	double hsvThresholdValue[] = {180, 255};
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
	//Step CV_erode0:
	//input
	cv::Mat cvErodeSrc = hsvThresholdOutput;
	cv::Mat cvErodeKernel;
	cv::Point cvErodeAnchor(-1, -1);
	double cvErodeIterations = 1.0;  // default Double
    int cvErodeBordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvErodeBordervalue(-1);
	cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, this->cvErodeOutput);
	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = cvErodeOutput;
	bool findContoursExternalOnly = false;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);
	//Step Filter_Contours0:
	//input

	if (findContoursOutput.size() == 0){
		return;
	}

// 	for (auto &cont : findContoursOutput){
// 		cv::Rect temp = cv::boundingRect(cv::Mat(cont));
// 		double ratio = temp.height/static_cast<double>(temp.width);
// 		if (ratio > 1.5 && ratio < 3.5){
// 			filterContoursOutput.push_back(cont);
// 			rectangles.push_back(temp);			
// 		}
// 	}

	for (int i = 0; i<rectangles.size()-1; i++){
		for (int j = i+1; j<rectangles.size(); j++){
			double rect_ratio = static_cast<double>(rectangles[i].area())/rectangles[j].area();
			if (rect_ratio > 0.8 && rect_ratio < 1.2){
				cv::Rect rect1;
				cv::Rect rect2;
				std::vector<cv::Point> contour1;
				std::vector<cv::Point> contour2;
				if (rectangles[i].x < rectangles[j].x){
					rect1 = rectangles[j];
					contour1 = filterContoursOutput[j];
					rect2 = rectangles[i];
					contour2 = filterContoursOutput[i];
				} else {
					rect1 = rectangles[i];
					contour1 = filterContoursOutput[i];
					rect2 = rectangles[j];
					contour2 = filterContoursOutput[j];
				}
				cv::Point farPoint1 = findFarthestPoint(contour1, false);
				cv::Point farPoint2 = findFarthestPoint(contour2, true);
				if (farPoint1.y < (rect1.tl().y+rect1.br().y)/2 && farPoint2.y > (rect2.tl().y+rect2.br().y)/2){
					rectangle_pairs.push_back({rectangles[i],rectangles[j]});				
				}
			}
		}
	}
//  //scp *.cpp *.h pi@10.8.30.11:RaspberryPiCamera;ssh -t pi@10.8.30.11 'cd RaspberryPiCamera;make clean;make install;make'


// 	for (auto rect_pair : rectangle_pairs){
// 		cv::rectangle(resizeImageOutput, rect_pair[0], {0,255,255}, 2);
// 		cv::rectangle(resizeImageOutput, rect_pair[1], {0,255,255}, 2);
// 	}

	contour_pairs.clear();
	rectangle_pairs.clear();
	rectangles.clear();
	
}	

bool GripPipeline::compareRectAreas(cv::Rect a, cv::Rect b){
	return static_cast<double>(a.area()) > static_cast<double>(b.area());
}

cv::Point GripPipeline::findFarthestPoint(std::vector<cv::Point> contour, bool direction/*true is left, false is right*/){
	cv::Point temp_point = contour[0];
	for (cv::Point iterator : contour){
		if ((!direction && iterator.x > temp_point.x) || (direction && iterator.x < temp_point.x)){
			temp_point = iterator;
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

