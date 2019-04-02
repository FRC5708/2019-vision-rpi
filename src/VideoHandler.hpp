#pragma once

#include <vector>

#include <opencv2/core.hpp>
#include <linux/videodev2.h>



class VideoReader {
	int camfd;
	void* currentBuffer;
	std::vector<void*> buffers;
	struct v4l2_buffer bufferinfo;

public:
	int width, height;

	void openReader(int width, int height, const char* file);
	cv::Mat getMat();

    void grabFrame(bool firstTime = false);

	void setExposureVals(bool isAuto, int exposure);
};

class VideoWriter {
	unsigned int vidsendsiz;
	int v4l2lo;

public:

	void openWriter(int width, int height, const char* file);
    void writeFrame(cv::Mat& frame);
};