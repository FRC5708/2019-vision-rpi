#include "vision.hpp"
#include "RedContourGrip.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

// this function from opencv/samples/cpp/tutorial_code/calib3d/camera_calibration/camera_calibration.cpp
using std::vector;
namespace cv {
	static double computeReprojectionErrors( const vector<Point3f>& objectPoints,
											const vector<Point2f>& imagePoints,
											const Mat& rvec, const Mat& tvec,
											const Mat& cameraMatrix , const Mat& distCoeffs) {
		vector<Point2f> imagePoints2;
		size_t totalPoints = 0;
		double totalErr = 0, err;

		projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints2);
		
		err = norm(imagePoints, imagePoints2, NORM_L2);

		size_t n = objectPoints.size();
		totalErr        += err*err;
		totalPoints     += n;

		return std::sqrt(totalErr/totalPoints);
	}
}

bool isImageTesting = false;
bool verboseMode = false;

namespace calib {
	cv::Mat cameraMatrix, distCoeffs;
	int width, height;
}


bool isNanOrInf(double in) {
	return isnan(in) || isinf(in);
}

struct AngularCoord {
	double x; double y;
};
AngularCoord toAngularCoord(double pixX, double pixY, double pixFocalLength, int pixImageWidth, int pixImageHeight) {
	double centeredX = pixX - pixImageWidth / 2;
	double centeredY = pixImageHeight / 2 - pixY;
	
	double rayLength = sqrt(centeredX*centeredX + centeredY*centeredY + pixFocalLength*pixFocalLength);
	double radX = atan2(centeredX, pixFocalLength);
	double radY = asin(centeredY / rayLength);
	
	 std::cout << "pixel: " << pixX << "," << pixY
	 << " angular coord: " << radX << ", " << radY << std::endl;
	return { radX, radY };
}

// from http://answers.opencv.org/question/16796/computing-attituderoll-pitch-yaw-from-solvepnp/?answer=52913#post-id-52913
cv::Vec3d getEulerAngles(cv::Mat &rotation) {
	cv::Mat rotCamerMatrix;
	cv::Rodrigues(rotation, rotCamerMatrix);

    cv::Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
	cv::Vec3d eulerAngles;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix(cv::Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
	return eulerAngles;
}

struct ProcessPointsResult {
	bool success;
	VisionData calcs;
	VisionDrawPoints drawPoints;
};

// all the constants
constexpr double radTapeOrientation = 14.5/180*M_PI;
constexpr double inchTapesWidth = 2;
constexpr double inchTapesLength = 5.5;
constexpr double inchInnerTapesApart = 8;
const double inchTapesHeight = inchTapesLength*cos(radTapeOrientation) + inchTapesWidth*sin(radTapeOrientation); // from bottom to top of tapes
const double inchTapeTopsApart = inchInnerTapesApart + 2*inchTapesWidth*cos(radTapeOrientation);
const double inchTapeBottomsApart = inchInnerTapesApart + 2*inchTapesLength*sin(radTapeOrientation);
const double inchOuterTapesApart = inchTapeTopsApart + 2*inchTapesLength*sin(radTapeOrientation); // from outermost edge
const double inchHatchTapesAboveGround = 2*12+7.5 - inchTapesHeight;
const double inchPortTapesAboveGround = 3*12+3.125 - inchTapesHeight;

// maximum difference in calculated and actual tape height above ground
constexpr double inchTapeHeightTolerance = 1000; // for testing

constexpr double inchMinDistance = 1;

// need precise values of these
constexpr double radCameraPitch = (30/180)*M_PI; // above horizontal
constexpr double inchCameraHeight = 20;//9.5; // from ground


// todo: use actual calibration data
// see: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
// right now, it assumes perfect pinhole camera

// these aren't used anymore
// for old cameras
constexpr double radFOV = (69.0/180.0)*M_PI;

// for new cameras
//constexpr double radFOV = (78.0/180.0)*M_PI;

// this function is obsolete, and only called in verbose mode
// this function uses hungarian notation for numbers. It's confusing otherwise
ProcessPointsResult processRects(cv::Rect left, cv::Rect right, int pixImageWidth, int pixImageHeight) {
	VisionData result;
	
	const double pixFocalLength = tan((M_PI_2) - radFOV/2) * sqrt(pow(pixImageWidth, 2) + pow(pixImageHeight, 2))/2; // pixels. Estimated from the camera's FOV spec.
	
	struct { AngularCoord tl, tr, bl, br; } radTapeQuad = {
		toAngularCoord(left.tl().x, left.tl().y, pixFocalLength, pixImageWidth, pixImageHeight),
		toAngularCoord(right.br().x, right.tl().y, pixFocalLength, pixImageWidth, pixImageHeight),
		toAngularCoord(left.tl().x, left.br().y, pixFocalLength, pixImageWidth, pixImageHeight),
		toAngularCoord(right.br().x, right.br().y, pixFocalLength, pixImageWidth, pixImageHeight)
	};
	assert(abs(radTapeQuad.tl.x - radTapeQuad.bl.x) < 0.00000001
		   && abs(radTapeQuad.tr.x - radTapeQuad.br.x) < 0.00000001);
	
	
	double inchLeftDistance = (inchTapesHeight /
							   (tan(radCameraPitch + radTapeQuad.tl.y) -
								tan(radCameraPitch + radTapeQuad.bl.y)));
	
	double inchRightDistance = inchTapesHeight /
	(tan(radCameraPitch + radTapeQuad.tr.y) -
	 tan(radCameraPitch + radTapeQuad.br.y));
	
	double radWidth = radTapeQuad.tr.x - radTapeQuad.tl.x;
	assert(radWidth > 0);
	
	
	// Uses law of sines to get the angle, A, between leftDistance and tapeWidth
	// then uses law of cosines to get the remaining side of the leftDistance-A-(tapeWidth/2) triangle
	// which is centerDistance
	double inchCenterDistanceShouldBe = sqrt(pow(inchLeftDistance, 2) + pow(inchOuterTapesApart/2, 2) -
							   inchLeftDistance*inchOuterTapesApart*
							   sqrt(1 - pow(sin(radWidth) / inchOuterTapesApart * inchRightDistance, 2))); // cos(A)
	
	// alternate version, for testing, that doesn't take into account the observed tape width
	double inchCenterDistance = sqrt(pow(inchLeftDistance, 2) + pow(inchOuterTapesApart/2, 2)
	 - (pow(inchLeftDistance, 2) + pow(inchOuterTapesApart, 2) - pow(inchRightDistance, 2)) / 2);

	double radWidthShouldBe = acos((pow(inchLeftDistance,2) + pow(inchRightDistance,2) - pow(inchOuterTapesApart,2))
	/ (2*inchRightDistance*inchLeftDistance));

	// in the ideal model, these are equal.
	double inchCalcTapesAboveGroundLeft  = inchLeftDistance  * tan(radCameraPitch + radTapeQuad.bl.y) + inchCameraHeight;
	double inchCalcTapesAboveGroundRight = inchRightDistance * tan(radCameraPitch + radTapeQuad.br.y) + inchCameraHeight;
	
	double inchCalcTapesAboveGround = (inchCalcTapesAboveGroundLeft + inchCalcTapesAboveGroundRight) / 2;

	std::cout << "distance: left: " << inchLeftDistance << " right: " << inchRightDistance << 
	" center: " << inchCenterDistance << " should be: " << inchCenterDistanceShouldBe <<
	"\nheight: " << inchCalcTapesAboveGround << " radWidth: " << radWidth << " should be: " << radWidthShouldBe <<
	"\nradHeight:L: " << radTapeQuad.tl.y - radTapeQuad.bl.y << " R: " << radTapeQuad.tr.y - radTapeQuad.br.y << '\n';

	if (inchCenterDistance > inchMinDistance/* &&
		abs(inchCalcTapesAboveGroundLeft - inchCalcTapesAboveGroundRight) < inchTapeHeightTolerance*/) {

		if (abs(inchCalcTapesAboveGround - inchHatchTapesAboveGround) < inchTapeHeightTolerance) {
			result.isPort = false;
		}
		else if (abs(inchCalcTapesAboveGround - inchPortTapesAboveGround) < inchTapeHeightTolerance) {
			result.isPort = true;
		}
		//else return { false, {}};
	}
	else return { false, {}};
	
	result.height = inchCalcTapesAboveGround;
	result.distance = inchCenterDistance;
	
	// the part of the width angle left of centerDistance
	// sin(A) / centerDistance == sin(leftAngle) / (inchOuterTapesApart / 2)
	double radLeftWidth = asin(sin(radWidth) / inchOuterTapesApart * inchRightDistance
						   / inchCenterDistance * (inchOuterTapesApart / 2));
	result.robotAngle = radTapeQuad.tl.x + radLeftWidth;
	// Let B = the angle between centerDistance and tapeApart/2.
	// tapeAngle = 90 - B. A + leftWidth + B = 180
	result.tapeAngle = (radLeftWidth + asin(sin(radWidth) / inchOuterTapesApart * inchRightDistance)) - M_PI_2;


	
	std::cout << "x:" << result.distance * sin(result.robotAngle) <<
	" y:" << result.distance * cos(result.robotAngle) <<
	" height:" << inchCalcTapesAboveGround << '\n';
	std::cout << "tapeAngle: " << result.tapeAngle << " robotAngle: " << result.robotAngle << '\n' << std::endl;

	return { true, result };
}

struct ContourCorners {
	cv::Point left, top, right, bottom;
	ContourCorners() : left(INT_MAX, INT_MAX), 
	top(INT_MAX, INT_MAX), 
	right(0, 0),
	bottom(0, 0) {}
};

ContourCorners getContourCorners(std::vector<cv::Point>& contour) {
	ContourCorners result;
	
	for (auto point : contour) {
		if (point.x < result.left.x) result.left = point;
		if (point.x > result.right.x) result.right = point;
		if (point.y < result.top.y) result.top = point;
		if (point.y > result.bottom.y) result.bottom = point;
	}
	return result;
}

bool matContainsNan(cv::Mat& in) {
	for (unsigned int i = 0; i < in.total(); ++i) {
		if (in.type() == CV_32F && isnan(in.at<float>(i))) return true;
		else if (in.type() == CV_64F && isnan(in.at<double>(i))) return true;
	}
	return false;
}

void drawVisionPoints(VisionDrawPoints& toDraw, cv::Mat& image) {
	for (int i = 0; i < 8; ++i) {
		cv::circle(image, toDraw.points[i], 1, cv::Scalar(0, 255, 0), 2);
	}
	for (int i = 8; i < 16; ++i) {
		cv::circle(image, toDraw.points[i], 1, cv::Scalar(0, 0, 255), 2);
	}
	for (int i = 18; i < 22; ++i) {
		int oppPoint = i + 1;
		if (oppPoint == 22) oppPoint = 18;
		cv::line(image, toDraw.points[i], toDraw.points[oppPoint], cv::Scalar(255, 0, 0), 1);
	}
	for (int i = 22; i < (int) (sizeof(VisionDrawPoints) / sizeof(cv::Point2f)); i += 2) {
		cv::line(image, toDraw.points[i], toDraw.points[i + 1], cv::Scalar(255, 0, 0), 2);
	}
	cv::line(image, toDraw.points[16], toDraw.points[17], cv::Scalar(0, 0, 255), 2);
	cv::circle(image, toDraw.points[16], 2, cv::Scalar(0, 255, 255), 4);
}

cv::Mat* debugDrawImage;
void showDebugPoints(VisionDrawPoints& toDraw) {
	if (!isImageTesting) return;
	cv::Mat drawOn = debugDrawImage->clone();
	
	drawVisionPoints(toDraw, drawOn);
	cv::namedWindow("projection");
	imshow("projection", drawOn);
	cv::waitKey(0);
	/*static int imgNum = 0;
	++imgNum;
	cv::imwrite("./debugimg_" + std::to_string(imgNum) + ".png", drawOn);*/
}

void invertPose(cv::Mat& rotation_vector, cv::Mat& translation_vector, cv::Mat& cameraRotationVector, cv::Mat& cameraTranslationVector) {
	cv::Mat R;
	cv::Rodrigues(rotation_vector, R);
	cv::Rodrigues(R.t(),cameraRotationVector);
	cameraTranslationVector = -R.t()*translation_vector;
}

ProcessPointsResult processPoints(ContourCorners left, ContourCorners right,
 int pixImageWidth, int pixImageHeight) {

	// There might be a bug in openCV that would require the focal length to be multiplied by 2.
	// Test this.

	// world coords: (0, 0, 0) at bottom center of tapes
	// up and right are positive. y-axis is vertical.
	std::vector<cv::Point3f> worldPoints = {
		cv::Point3f(-inchOuterTapesApart/2, inchTapesWidth*sin(radTapeOrientation), 0),
		cv::Point3f(inchOuterTapesApart/2, inchTapesWidth*sin(radTapeOrientation), 0),
		cv::Point3f(-inchTapeTopsApart/2, inchTapesHeight, 0),
		cv::Point3f(inchTapeTopsApart/2, inchTapesHeight, 0),
		cv::Point3f(-inchInnerTapesApart/2, inchTapesLength*cos(radTapeOrientation), 0),
		cv::Point3f(inchInnerTapesApart/2, inchTapesLength*cos(radTapeOrientation), 0),
		cv::Point3f(-inchTapeBottomsApart/2, 0, 0),
		//cv::Point3f(inchTapeBottomsApart/2, 0, 0)
	};
	std::vector<cv::Point2f> imagePoints = {
		left.left, right.right, left.top, right.top,
		left.right, right.left, left.bottom//, right.bottom
	};

	cv::Mat rvec, tvec, rotation, translation;
	cv::solvePnP(worldPoints, imagePoints, calib::cameraMatrix, calib::distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
	invertPose(rvec, tvec, rotation, translation);
	
	assert(tvec.type() == CV_64F && rvec.type() == CV_64F && rotation.type() == CV_64F && translation.type() == CV_64F);
	
	if (matContainsNan(translation) || matContainsNan (rotation)) {
		std::cout << "solvePnP returned NaN!\n";
		return { false, {}};
	} 

	double pixError = cv::computeReprojectionErrors(worldPoints, imagePoints, rvec, tvec, calib::cameraMatrix, calib::distCoeffs);

	cv::Vec3d angles = getEulerAngles(rvec);
	
	// x is right postive, y is forwards positive
	double inchRobotX = translation.at<double>(0);
	double inchRobotY = translation.at<double>(2);
	double inchCalcTapesAboveCamera = translation.at<double>(1);

	if (verboseMode) {
		std::cout << "\nerror:" << pixError << " pitch:" << angles[0] << " yaw:" << angles[1] << " roll:" << angles[2] 
		<< "\ntrans: " << translation << "\ntvec: " << tvec <<
		"\nx:" << inchRobotX << " y:" << inchRobotY << " height:" << inchCalcTapesAboveCamera << '\n';
	}

	VisionData result;
	result.distance = sqrt(pow(inchRobotX, 2) + pow(inchRobotY, 2));

	result.tapeAngle = -atan2(inchRobotX, inchRobotY);
	result.robotAngle = -asin(tvec.at<double>(0) / result.distance);

	if (isNanOrInf(result.distance) || isNanOrInf(result.robotAngle) || isNanOrInf(result.tapeAngle)) {
		std::cout << "encountered NaN or Infinity" << std::endl;
		return { false, {} };
	}

	double pixMaxError = std::max(3, 
		((left.bottom.y - left.top.y) + (right.bottom.y - right.top.y))/2 / 6);
	//constexpr double radMaxCameraPitch = 40.0/180.0*M_PI;
	constexpr double degMaxRoll = 20; // degrees
	constexpr double inchMinDistance = 10;
	
	//double radReferencePitch = fmod((radPitch + 2*M_PI), M_PI); // make positive
	//if (radReferencePitch > M_PI_2) radReferencePitch = M_PI - radReferencePitch;

	if (pixError > pixMaxError ||
	result.distance < inchMinDistance ||
	//radReferencePitch > radMaxCameraPitch || 
	fabs(angles[2]) > degMaxRoll) {
		std::cout << "result rejected" << std::endl;
		return { false, {}};
	}

	VisionDrawPoints draw;
	std::copy(imagePoints.begin(), imagePoints.end(), draw.points);
	
	constexpr float CROSSHAIR_LENGTH = 4;
	worldPoints.insert(worldPoints.end(), {
		cv::Point3f(0, 0, CROSSHAIR_LENGTH), cv::Point3f(0, 0, -CROSSHAIR_LENGTH),
		
		cv::Point3f(CROSSHAIR_LENGTH, CROSSHAIR_LENGTH, 0), cv::Point3f(-CROSSHAIR_LENGTH, CROSSHAIR_LENGTH, 0),
		cv::Point3f(-CROSSHAIR_LENGTH, -CROSSHAIR_LENGTH, 0), cv::Point3f(CROSSHAIR_LENGTH, -CROSSHAIR_LENGTH, 0),
		
		cv::Point3f(-CROSSHAIR_LENGTH, 0, 0), cv::Point3f(CROSSHAIR_LENGTH, 0, 0),
		cv::Point3f(0, -CROSSHAIR_LENGTH, 0), cv::Point3f(0, CROSSHAIR_LENGTH, 0)
	});
	
	cv::Mat projPoints;
	cv::projectPoints(worldPoints, rvec, tvec, calib::cameraMatrix, calib::distCoeffs, projPoints);
	assert(projPoints.type() == CV_32FC2);
	std::copy(projPoints.begin<cv::Point2f>(), projPoints.end<cv::Point2f>(), draw.points + 8);
	
	if (isImageTesting) showDebugPoints(draw);
	
	return { true, result, draw };
}

void testSideways() {
	for (int leftmost = 0; leftmost < 402-231; leftmost += 10) {
		std::cout << "center: " << leftmost + 231/2 << std::endl;
		cv::Rect left(leftmost, 31, 10, 117);
		cv::Rect right(leftmost+231-10, 31, 10, 117);
		processRects(left, right, 402, 800);
	}
}

std::vector<VisionTarget> processContours(
	std::vector<std::vector<cv::Point> >* contours, int imgWidth, int imgHeight) {
std::vector<cv::Rect> rects;
	std::vector<ContourCorners> contourCorners;

	const float minRectWidth = 10; //pixels 
	const float minRectHeight= 10;
	for (auto i : *contours) {
		cv::Rect rect = cv::boundingRect(i);
		if (rect.width >= minRectWidth && rect.height >= minRectHeight) {
			rects.push_back(rect);
			contourCorners.push_back(getContourCorners(i));
		}
	}

	const float rectSizeDifferenceTolerance = 0.5; // fraction of width/height
	const float rectYDifferenceTolerance = 0.5;
	const float rectDistanceTolerance = 10; // multiplier of the width of one rectangle, that the whole vision target can be

	if (verboseMode) for (auto rect : rects) {
		std::cout << "found rect: x:" << rect.x << ",y:" << rect.y << ",w:" << rect.width << ",h:" << rect.height << std::endl;
	}

	std::vector<VisionTarget> results;
	
	// find rects that are close enough in size and distance
	for (unsigned int i = 0; i < rects.size(); ++i) {
		for (unsigned int j = 0; j < rects.size(); ++j) {
			cv::Rect& left = rects[i];
			cv::Rect& right = rects[j];

			if (left != right &&
				left.br().x < right.tl().x &&
				left.tl().x + (left.width + right.width) / 2 * rectDistanceTolerance > right.br().x &&
			    abs(left.width - right.width) < rectSizeDifferenceTolerance * (left.width + right.width) / 2 &&
				abs(left.height - right.height) < rectSizeDifferenceTolerance * (left.width + right.width) / 2 &&
				abs(left.br().y - right.br().y) < rectYDifferenceTolerance * (left.height + right.height) / 2) {
				try {
				// keep around old output for debugging
				//if (verboseMode) processRects(left, right, imgWidth, imgHeight);

				ProcessPointsResult result = processPoints(
					contourCorners[i], contourCorners[j], imgWidth, imgHeight);

				if (result.success) {
					results.push_back({ result.calcs, result.drawPoints, left, right });
				}
				}
				catch (const cv::Exception e) {
					if (isImageTesting) throw;
					std::cerr << e.msg << std::endl;
				}
			}
		}
	}
	// lowest distance first
	std::sort(results.begin(), results.end(), [](VisionTarget a, VisionTarget b) -> bool {
		return a.calcs.distance < b.calcs.distance;
	});
	
	return results;
}

std::vector<VisionTarget> doVision(cv::Mat image) {
	if (isImageTesting) debugDrawImage = &image;

	grip::RedContourGrip finder;
	finder.Process(image);
	
	auto results1 = processContours(finder.GetBrightContours(), image.cols, image.rows);
	auto results2 = processContours(finder.GetRedContours(), image.cols, image.rows);

	results1.insert(results1.begin(), results2.begin(), results2.end());
	return results1;
}


