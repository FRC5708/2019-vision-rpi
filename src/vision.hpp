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

struct VisionTarget {
	VisionData calcs;
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
