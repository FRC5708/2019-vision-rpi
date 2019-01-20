#pragma once

#include <opencv2/core.hpp>

struct VisionData {
	bool isPort;
	double distance;
	
	// Angles: left/down=negative, right/up=positive
	double tapeAngle; // angle of tapes from directly facing robot
	double robotAngle; // angle of robot from directly facing tapes
};

struct VisionTarget {
	VisionData calcs;
	cv::Rect left, right;
};

std::vector<VisionTarget> doVision(cv::Mat image);
