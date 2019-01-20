#pragma once
#include <unistd.h>
#include <opencv2/core.hpp>

#include "vision.hpp"

class Streamer {
	FILE* videoFifo;
	
public:
	Streamer(int width, int height);
	
	void writeFrame(cv::Mat image, std::vector<VisionTarget>& toDraw);
};
