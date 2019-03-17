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
	
	void launchGStreamer(const char* recieveAddress, int bitrate);
	const char* prevRecvAddr;

	pid_t gstreamerPID = 0, ffmpegPID = 0;
	int servFd;
public:
	int width, height;
	
	void start(int width, int height);
	volatile bool handlingLaunchRequest = false;
	
	void writeFrame(cv::Mat image, std::vector<VisionTarget>& toDraw);
	
	void relaunchGStreamer();
	
	cv::Mat getBGRFrame();

	void run(std::function<void(void)> frameNotifier); // run thread
	void launchFFmpeg(); // for loopback video
};
extern int clientFd; 