#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

struct VisionData {
	double height;
	bool isPort;
	double distance;
	
	// Angles: left/down=negative, right/up=positive
	
	// angle of tapes from directly facing robot.
	// Positive when robot is left of tapes while facing them from the front.
	double tapeAngle;
	
	// angle of robot from directly facing tapes.
	// Positive when tapes are on the right side of the camera's image.
	double robotAngle;
};

struct VisionDrawPoints {
	// First, 8 points on corners of vision targets.
	// Then those points, reprojected using the target location.
	// the following six points should be drawn as lines:
	// the next two lines form a crosshair on the vision target
	// the last line goes through that crosshair along the z-axis
	// (last line should be a different color)
	
	cv::Point2f points[22];
};
void drawVisionPoints(VisionDrawPoints& toDraw, cv::Mat& image);

struct VisionTarget {
	VisionData calcs;
	VisionDrawPoints drawPoints;
	cv::Rect left, right;
};

void testSideways();
std::vector<VisionTarget> doVision(cv::Mat image);

extern bool isImageTesting;
extern bool verboseMode;

namespace calib {
	extern cv::Mat cameraMatrix, distCoeffs;
	extern int width, height;
}
