#include "CargoGripPipeline.h"

CargoGripPipeline::CargoGripPipeline() {
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void CargoGripPipeline::Process(cv::Mat& source0){
	//Step Blur0:
	//input
	cv::Mat blurInput = source0;
	BlurType blurType = BlurType::BOX;
	double blurRadius = 45.04504504504504;  // default Double
	blur(blurInput, blurType, blurRadius, this->blurOutput);
	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThresholdInput = blurOutput;
	double hsvThresholdHue[] = {0.0, 23.344709897610933};
	double hsvThresholdSaturation[] = {71.08812949640287, 220.1877133105802};
	double hsvThresholdValue[] = {240.78237410071944, 255.0};
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
	//Step Find_Blobs0:
	//input
	cv::Mat findBlobsInput = hsvThresholdOutput;
	double findBlobsMinArea = 1;  // default Double
	double findBlobsCircularity[] = {0.5575539568345323, 1.0};
	bool findBlobsDarkBlobs = false;  // default Boolean
	findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, this->findBlobsOutput);

	if (findBlobsOutput.size > 0){
		findBlobsOutput.Sort(0,findBlobsOutput.size,compareBlobs);
		Smartdashboard::PutNumber("Cargo Mid X", findBlobsOutput[0].x);
	}
}

bool CargoGripPipeline::compareBlobs(cv::KeyPoint a, cv::KeyPoint b){
	return a.size() < b.size();
}

/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
cv::Mat* CargoGripPipeline::GetBlurOutput(){
	return &(this->blurOutput);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* CargoGripPipeline::GetHsvThresholdOutput(){
	return &(this->hsvThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Find_Blobs.
 * @return BlobsReport output from Find_Blobs.
 */
std::vector<cv::KeyPoint>* CargoGripPipeline::GetFindBlobsOutput(){
	return &(this->findBlobsOutput);
}
	/**
	 * Softens an image using one of several filters.
	 *
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	void CargoGripPipeline::blur(cv::Mat &input, BlurType &type, double doubleRadius, cv::Mat &output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type) {
			case BOX:
				kernelSize = 2 * radius + 1;
				cv::blur(input,output,cv::Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				cv::medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				cv::bilateralFilter(input, output, -1, radius, radius);
				break;
        }
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
	void CargoGripPipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Detects groups of pixels in an image.
	 *
	 * @param input The image on which to perform the find blobs.
	 * @param minArea The minimum size of a blob that will be found.
	 * @param circularity The minimum and maximum circularity of blobs that will be found.
	 * @param darkBlobs The boolean that determines if light or dark blobs are found.
	 * @param blobList The output where the MatOfKeyPoint is stored.
	 */
	//void findBlobs(Mat *input, double *minArea, double circularity[2],
		//bool *darkBlobs, vector<KeyPoint> *blobList) {
	void CargoGripPipeline::findBlobs(cv::Mat &input, double minArea, double circularity[], bool darkBlobs, std::vector<cv::KeyPoint> &blobList) {
		blobList.clear();
		cv::SimpleBlobDetector::Params params;
		params.filterByColor = 1;
		params.blobColor = (darkBlobs ? 0 : 255);
		params.minThreshold = 10;
		params.maxThreshold = 220;
		params.filterByArea = true;
		params.minArea = minArea;
		params.filterByCircularity = true;
		params.minCircularity = circularity[0];
		params.maxCircularity = circularity[1];
		params.filterByConvexity = false;
		params.filterByInertia = false;
		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		detector->detect(input, blobList);
	}
