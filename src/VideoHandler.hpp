#pragma once
#include <unistd.h>
#include <opencv2/core.hpp>
#include <mutex>
#include <condition_variable>

#include "vision.hpp"

class VideoHandler {
	FILE* videoInFifo;
	FILE* videoOutFifo;
	
	std::mutex waitLock;
	std::condition_variable condition;

	int width, height;
	
public:
	VideoHandler(int width, int height);
	
	cv::Mat readFrame();

	cv::Mat drawFrame(cv::Mat image, std::vector<VisionTarget>& toDraw);
	void writeFrame(cv::Mat image);
};
