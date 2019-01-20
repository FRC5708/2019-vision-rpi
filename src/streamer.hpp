#pragma once
#include <unistd.h>
#include <opencv2/core.hpp>
#include <mutex>
#include <condition_variable>

#include "vision.hpp"

class Streamer {
	FILE* videoFifo;
	
	std::mutex waitLock;
	std::condition_variable condition;
	
	void _writeFrame();
	
	cv::Mat image;
	std::vector<VisionTarget> toDraw;
public:
	Streamer(int width, int height);
	
	void writeFrame(cv::Mat image, std::vector<VisionTarget>& toDraw);
	
	void operator()(); // run thread
};
