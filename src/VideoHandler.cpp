#include "VideoHandler.hpp"

#include <unistd.h>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <chrono>
#include <thread>

#include <opencv2/imgproc.hpp>

using std::cout; using std::endl; using std::string;


int getMaxPipeSize() {
	FILE* maxSizeFile;
	if ((maxSizeFile = fopen("/proc/sys/fs/pipe-max-size", "r")) == nullptr) {
		perror("could not read allowed pipe size");
		return 65536;
	}
	char buf[100];
	fread(buf, sizeof(buf), 1, maxSizeFile);
	return atoi(buf);
}
void embiggenPipe(FILE* pipeFile, int targetSize) {
	static int maxPipeSize = getMaxPipeSize();

	if (targetSize > maxPipeSize) targetSize = maxPipeSize;

	if (fcntl(fileno(pipeFile), F_SETPIPE_SZ, targetSize) < 0) {
		perror("could not set pipe size");
	}
}

VideoHandler::VideoHandler(int width, int height) {
	this -> width = width; this->height = height;
	
	string filesDir = string(getenv("HOME")) + "/vision_files";
	string videoInPath = filesDir + "/camera-stream";
	string videoOutPath = filesDir + "/out-stream";

	system(("mkfifo " + videoInPath).c_str());
	system(("mkfifo " + videoOutPath).c_str());

	string ffmpegCommand = "ffmpeg -f v4l2 -video_size " + std::to_string(width) + "x" + std::to_string(height) + 
	" -i /dev/video0 -f rawvideo -pixel_format bgr24 " + videoInPath + " &";
	cout << "> " << ffmpegCommand << endl;
	system(ffmpegCommand.c_str());
	
#ifdef __linux__
	string codec = "omxh264enc";
	string gstreamCommand = "gst-launch-1.0";
#elif defined __APPLE__
	string codec = "omxh264enc"; //TODO: ADDME
	string gstreamCommand = "gst-launch-1.0";
#endif
	
	string recieveAddress = "10.126.58.95";
	int target_bitrate = 3000000;
	int port=1234;
	
	std::stringstream command;
	command << gstreamCommand << " filesrc location=" << videoOutPath << 
	"video/x-raw,width=" << width << ",height=" << height << ",format=BGR ! videoconvert ! queue ! " 
	<< codec << " target-bitrate=" << target_bitrate << 
	" control-rate=variable ! video/x-h264, width=" << width << ",height=" << height << ",framerate=30/1,profile=high ! rtph264pay ! gdppay ! udpsink"
	<< " host=" << recieveAddress << " port=" << port << " &";

	cout << command.str() << endl;
	system(command.str().c_str());

	videoInFifo = fopen(videoInPath.c_str(), "r");
	videoOutFifo = fopen(videoOutPath.c_str(), "w");

	embiggenPipe(videoInFifo, width*height*3);
	embiggenPipe(videoOutFifo, width*height*3);

	// play stream with:
	// ADDME
	// It needs to be started BEFORE this program for stupid reasons
}

cv::Mat VideoHandler::readFrame() {
	cv::Mat frame(height, width, CV_8UC3);
	fread(frame.data, frame.total()*frame.elemSize(), 1, videoInFifo);
	return frame;
}

cv::Mat VideoHandler::drawFrame(cv::Mat image, std::vector<VisionTarget>& toDraw) {
	cv::Mat drawnOn = image.clone();
	
	for (auto i = toDraw.begin(); i < toDraw.end(); ++i) {
		
		cv::Scalar color;
		if (i == toDraw.begin()) color = cv::Scalar(0, 255, 0);
		else color = cv::Scalar(255, 0, 0);
		
		cv::rectangle(drawnOn, toDraw[0].right, color);
		cv::rectangle(drawnOn, toDraw[0].left, color);
	}
	return drawnOn;
}
void VideoHandler::writeFrame(cv::Mat image) {
	fwrite(image.data, image.total() * image.elemSize(), 1, videoOutFifo);
}
