#include "streamer.hpp"
#include "DataComm.h"

#include <string>
#include <iostream>
#include <fcntl.h>
#include <chrono>
#include <thread>

#include <opencv2/imgproc.hpp>

#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>

using std::cout; using std::endl; using std::string;

pid_t runCommandAsync(const std::string& cmd, int closeFd) {
	string execCmd = ("exec " + cmd);
	cout << "> " << execCmd << endl;
	pid_t pid = fork();
	
	if (pid == 0) {
		// child
		close(closeFd);
		
		const char *argv[] = {
			"/bin/sh",
			"-c",
			execCmd.c_str(),
			nullptr
		};
		
		execv("/bin/sh", (char *const* )argv);
		exit(127);
	}
	else return pid;
}

void Streamer::relaunchGStreamer() {
	if (!handlingLaunchRequest) launchGStreamer(prevRecvAddr, 1000000);
}

void Streamer::launchGStreamer(const char* recieveAddress, int bitrate) {
	prevRecvAddr = recieveAddress;
	cout << "launching GStreamer, targeting " << recieveAddress << endl;
	
#ifdef __linux__
	string codec = "omxh264enc";
	string gstreamCommand = "gst-launch-1.0 v4l2src device=/dev/video2";
#elif defined __APPLE__
	string codec = "omxh264enc";
	string gstreamCommand = "/usr/local/bin/gst-launch-1.0 autovideosrc";
#endif
	
	//int target_bitrate = 3000000;
	int port=5809;
	
	std::stringstream command;
	command << gstreamCommand << " ! videoscale ! videoconvert ! queue ! " << codec << " target-bitrate=" << bitrate <<
	" control-rate=variable ! video/x-h264, width=" << width << ",height=" << height << ",framerate=30/1,profile=high ! rtph264pay ! gdppay ! udpsink"
	<< " host=" << recieveAddress << " port=" << port;

	string strCommand = command.str();
	
	gstreamerPID = runCommandAsync(strCommand, servFd);
}

void Streamer::launchFFmpeg() {
	ffmpegPID = runCommandAsync(
		"ffmpeg -f v4l2 -pix_fmt yuyv422 -video_size  800x448 -i /dev/video0 -f v4l2 /dev/video1 -f v4l2 /dev/video2"
	, servFd);
}

// https://gist.github.com/thearchitect/96ab846a2dae98329d1617e538fbca3c
class V4lwriter {
	int vidsendsiz;
	int v4l2lo;
	int camfd;
	void* readBuffer;
	struct v4l2_buffer bufferinfo;
	
public:
	int width, height;

	static V4lwriter instance;
	
	void openReader() {
		// http://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/
		// https://jayrambhia.com/blog/capture-v4l2
		
		camfd = open("/dev/video0", O_RDWR);
		if (camfd == -1) {
			perror("open");
			exit(1);
		}
		
		struct v4l2_capability cap;
		if(ioctl(camfd, VIDIOC_QUERYCAP, &cap) < 0){
			perror("VIDIOC_QUERYCAP");
			exit(1);
		}
		if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
			fprintf(stderr, "The device does not handle single-planar video capture.\n");
			exit(1);
		}
		
		struct v4l2_format format;
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		format.fmt.pix.width = width;
		format.fmt.pix.height = height;
		
		if(ioctl(camfd, VIDIOC_S_FMT, &format) < 0){
			perror("VIDIOC_S_FMT");
			exit(1);
		}

		struct v4l2_requestbuffers bufrequest;
		bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		bufrequest.memory = V4L2_MEMORY_MMAP;
		bufrequest.count = 1;
		
		if(ioctl(camfd, VIDIOC_REQBUFS, &bufrequest) < 0){
			perror("VIDIOC_REQBUFS");
			exit(1);
		}
		memset(&bufferinfo, 0, sizeof(bufferinfo));
		
		bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		bufferinfo.memory = V4L2_MEMORY_MMAP;
		bufferinfo.index = 0;
		
		if(ioctl(camfd, VIDIOC_QUERYBUF, &bufferinfo) < 0){
			perror("VIDIOC_QUERYBUF");
			exit(1);
		}

		readBuffer = mmap(
			NULL,
			bufferinfo.length,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			camfd,
			bufferinfo.m.offset
		);
		if(readBuffer == MAP_FAILED){
			perror("mmap");
			exit(1);
		}
		memset(readBuffer, 0, bufferinfo.length);

		// Activate streaming
		int type = bufferinfo.type;
		if(ioctl(camfd, VIDIOC_STREAMON, &type) < 0){
			perror("VIDIOC_STREAMON");
			exit(1);
		}
	}
	void grabFrame() {
		// Put the buffer in the incoming queue.
		if(ioctl(camfd, VIDIOC_QBUF, &bufferinfo) < 0){
			perror("VIDIOC_QBUF");
			exit(1);
		}
		
		// The buffer's waiting in the outgoing queue.
		if(ioctl(camfd, VIDIOC_DQBUF, &bufferinfo) < 0){
			perror("VIDIOC_DQBUF");
			exit(1);
		}
	}
	cv::Mat getMat() {
		return cv::Mat(height, width, CV_8UC2, readBuffer);
	}
	
	void openWriter() {
		
		
		v4l2lo = open("/dev/video2", O_WRONLY);
		if(v4l2lo < 0) {
			std::cout << "Error opening v4l2l device: " << strerror(errno);
			exit(-2);
		}
		struct v4l2_format v;
		int t;
		v.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		t = ioctl(v4l2lo, VIDIOC_G_FMT, &v);
		if( t < 0 ) {
			exit(t);
		}
		v.fmt.pix.width = width;
		v.fmt.pix.height = height;
		v.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		vidsendsiz = width * height * 2;
		v.fmt.pix.sizeimage = vidsendsiz;
		t = ioctl(v4l2lo, VIDIOC_S_FMT, &v);
		if( t < 0 ) {
			exit(t);
		}
	}
	
	void writeFrame(cv::Mat& frame) {
		std::cout << "writing frame" << std::endl;
		assert(frame.total() * frame.elemSize() == vidsendsiz);
		
		if (write(v4l2lo, frame.data, vidsendsiz) == -1) {
			perror("writing frame");
		}
	}
};
V4lwriter V4lwriter::instance;

void Streamer::start(int width, int height) {
	this->width = width; this->height = height;

	std::thread([this]() {
		
		servFd = socket(AF_INET6, SOCK_STREAM, 0);
		if (servFd < 0) {
			perror("socket");
			return;
		}
		
		int flag = 1;
		if (setsockopt(servFd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag)) == -1) {
			perror("setsockopt");
		}
		
		struct sockaddr_in6 servAddr;
		memset(&servAddr, 0, sizeof(servAddr));
		
		servAddr.sin6_family = AF_INET6;
		servAddr.sin6_addr = in6addr_any;
		servAddr.sin6_port = htons(5807); //Port that resets magic
	
		if (bind(servFd, (struct sockaddr*)&servAddr, sizeof(servAddr)) == -1) {
			perror("bind");
			return;
		}
		if (listen(servFd, 10)== -1) {
			perror("listen");
			return;
		}
		
		while (true) {
			struct sockaddr_in6 clientAddr;
			socklen_t clientAddrLen = sizeof(clientAddr);
			int clientFd = accept(servFd, (struct sockaddr*) &clientAddr, &clientAddrLen);
			if (clientFd < 0) {
				perror("accept");
				continue;
			}

			handlingLaunchRequest = true;

			if (gstreamerPID > 0) {
				cout << "killing previous instance: " << gstreamerPID << "   " << endl;
				if (kill(gstreamerPID, SIGTERM) == -1) {
					perror("kill");
				}
				waitpid(gstreamerPID, nullptr, 0);
			}
			char bitrate[16];
			ssize_t len = read(clientFd, bitrate, sizeof(bitrate));
			bitrate[len] = '\0';
			
			const char message[] = "Launching remote GStreamer...\n";
			if (write(clientFd, message, sizeof(message)) == -1) {
				perror("write");
			}

			if (close(clientFd) == -1) perror("close");

			

			// wait for client's gstreamer to initialize
			sleep(2);

			char strAddr[INET6_ADDRSTRLEN];
			getnameinfo((struct sockaddr *) &clientAddr, sizeof(clientAddr), strAddr,sizeof(strAddr),
    		0,0,NI_NUMERICHOST);
			launchGStreamer(strAddr, atoi(bitrate));

			cout << "Starting UDP stream..." << endl;
			computer_udp = DataComm(strAddr);
			computer_udp_exists=true;

			handlingLaunchRequest = false;
		}
	}).detach();
	V4lwriter::instance.width = width;
	V4lwriter::instance.height = height;

	V4lwriter::instance.openWriter();
	V4lwriter::instance.openReader();
}

void Streamer::_writeFrame() {
	cv::Mat drawnOn = image.clone();
	//std::cout << "writing frame" << std::endl;
	
	for (auto i = toDraw.begin(); i < toDraw.end(); ++i) {
		drawVisionPoints(i->drawPoints, drawnOn);
	}
	
	V4lwriter::instance.writeFrame(drawnOn);
}

void Streamer::writeFrame(cv::Mat image, std::vector<VisionTarget>& toDraw) {
	this->image = image; this->toDraw = toDraw;
	
	/*waitLock.unlock();
	condition.notify_all();*/
	_writeFrame();
}

cv::Mat Streamer::getBGRFrame() {
	cv::Mat frame;
	cvtColor(V4lwriter::instance.getMat(), frame, cv::COLOR_YUV2BGR_YUYV);
	return frame;
}

void Streamer::run() {
	/*while (true) {
		std::unique_lock<std::mutex> uniqueLock(waitLock);
		condition.wait(uniqueLock);
		_writeFrame();
	}*/
	while (true) {
		V4lwriter::instance.grabFrame();
		cv::Mat drawnOn = V4lwriter::instance.getMat().clone();

		for (auto i = toDraw.begin(); i < toDraw.end(); ++i) {
			drawVisionPoints(i->drawPoints, drawnOn);
		}
		
		V4lwriter::instance.writeFrame(drawnOn);
	}
}
