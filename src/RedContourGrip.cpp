#include "RedContourGrip.hpp"

namespace grip {

RedContourGrip::RedContourGrip() {
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
//#define RED
#define GREEN

void myThreshold(cv::Mat& input, cv::Mat& bright, cv::Mat& red) {
	assert(input.type() == CV_8UC3);
	bright.create(input.rows, input.cols, CV_8U);
	red.create(input.rows, input.cols, CV_8U);

	#pragma omp parallel for
	for (unsigned int i = 0; i < input.total(); ++i) {
		cv::Vec3b px = input.at<cv::Vec3b>(i);
#ifdef RED
		bright.data[i] = (px[2] > 190) * 255;
		red.data[i] = (px[2] > 130 && px[2] - 30 > px[0] && px[2] - 30 > px[1]) * 255;
#elif defined(GREEN)
		bright.data[i] = (px[1] > 190) * 255;
		red.data[i] = (px[1] > 130 && px[1] - 30 > px[0] && px[1] - 30 > px[2]) * 255;
#else
		static_assert(false, "not red or green");
#endif
	}
}

void RedContourGrip::Process(cv::Mat& source0){
	//Step RGB_Threshold0:
	//input
	/*cv::Mat rgbThresholdInput = source0;
	double rgbThresholdRed[] = {181.16007194244605, 255.0};
	double rgbThresholdGreen[] = {0.0, 255.0};
	double rgbThresholdBlue[] = {0.0, 255.0};
	rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, this->rgbThresholdOutput);
	*/
	cv::Mat bright, red;
	myThreshold(source0, bright, red);
	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = rgbThresholdOutput;
	bool findContoursExternalOnly = false;  // default Boolean

	//Step Filter_Contours0:
	//input
	double filterContoursMinArea = 100;  // default Double
	double filterContoursMinPerimeter = 0;  // default Double
	double filterContoursMinWidth = 5.0;  // default Double
	double filterContoursMaxWidth = 1000;  // default Double
	double filterContoursMinHeight = 15.0;  // default Double
	double filterContoursMaxHeight = 1000;  // default Double
	double filterContoursSolidity[] = {70, 100};
	double filterContoursMaxVertices = 1000000;  // default Double
	double filterContoursMinVertices = 0;  // default Double
	double filterContoursMinRatio = 0;  // default Double
	double filterContoursMaxRatio = 1000;  // default Double
	
	std::vector<std::vector<cv::Point> > unfilteredBright, unfilteredRed;
	
	/*#pragma omp parallel
	#pragma omp single
	{
		#pragma omp task*/
		findContours(bright, findContoursExternalOnly, unfilteredBright);
		filterContours(unfilteredBright, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, this->brightContours);
		
		//#pragma omp task
		findContours(red, findContoursExternalOnly, unfilteredRed);
		filterContours(unfilteredRed, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, this->redContours);
	//}
}

/**
 * This method is a generated getter for the output of a RGB_Threshold.
 * @return Mat output from RGB_Threshold.
 */
cv::Mat* RedContourGrip::GetRgbThresholdOutput(){
	return &(this->rgbThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* RedContourGrip::GetBrightContours(){
	return &(this->brightContours);
}
/**
 * This method is a generated getter for the output of a Filter_Contours.
 * @return ContoursReport output from Filter_Contours.
 */
std::vector<std::vector<cv::Point> >* RedContourGrip::GetRedContours(){
	return &(this->redContours);
}
	/**
	 * Segment an image based on color ranges.
	 *
	 * @param input The image on which to perform the RGB threshold.
	 * @param red The min and max red.
	 * @param green The min and max green.
	 * @param blue The min and max blue.
	 * @param output The image in which to store the output.
	 */
	void RedContourGrip::rgbThreshold(cv::Mat &input, double red[], double green[], double blue[], cv::Mat &output) {
		cv::cvtColor(input, output, cv::COLOR_BGR2RGB);
		cv::inRange(output, cv::Scalar(red[0], green[0], blue[0]), cv::Scalar(red[1], green[1], blue[1]), output);
	}

	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
	void RedContourGrip::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
		std::vector<cv::Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
		int method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
	}


	/**
	 * Filters through contours.
	 * @param inputContours is the input vector of contours.
	 * @param minArea is the minimum area of a contour that will be kept.
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept.
	 * @param minWidth minimum width of a contour.
	 * @param maxWidth maximum width.
	 * @param minHeight minimum height.
	 * @param maxHeight  maximimum height.
	 * @param solidity the minimum and maximum solidity of a contour.
	 * @param minVertexCount minimum vertex Count of the contours.
	 * @param maxVertexCount maximum vertex Count.
	 * @param minRatio minimum ratio of width to height.
	 * @param maxRatio maximum ratio of width to height.
	 * @param output vector of filtered contours.
	 */
	void RedContourGrip::filterContours(std::vector<std::vector<cv::Point> > &inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double solidity[], double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, std::vector<std::vector<cv::Point> > &output) {
		std::vector<cv::Point> hull;
		output.clear();
		for (std::vector<cv::Point> contour: inputContours) {
			cv::Rect bb = boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			double area = cv::contourArea(contour);
			if (area < minArea) continue;
			//if (arcLength(contour, true) < minPerimeter) continue;
			cv::convexHull(cv::Mat(contour, true), hull);
			double solid = 100 * area / cv::contourArea(hull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			//if (contour.size() < minVertexCount || contour.size() > maxVertexCount)	continue;
			//double ratio = (double) bb.width / (double) bb.height;
			//if (ratio < minRatio || ratio > maxRatio) continue;
			output.push_back(contour);
		}
	}



} // end grip namespace

