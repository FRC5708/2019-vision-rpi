#include "streamer.hpp"

#include <string>
#include <iostream>
#include <fcntl.h>
#include <chrono>
#include <thread>

#include <opencv2/imgproc.hpp>

#include <unistd.h>
#include <signal.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using std::cout; using std::endl; using std::string;


void Streamer::launchGStreamer(const char* recieveAddress) {
	cout << "launching GStreamer, targeting " << recieveAddress << endl;
	
#ifdef __linux__
	string codec = "omxh264enc";
	string gstreamCommand = "gst-launch-1.0 v4l2src device=/dev/video2";
#elif defined __APPLE__
	string codec = "omxh264enc";
	string gstreamCommand = "/usr/local/bin/gst-launch-1.0 autovideosrc";
#endif
	
	int target_bitrate = 3000000;
	int port=1234;
	
	std::stringstream command;
	command << gstreamCommand << " ! videoscale ! videoconvert ! queue ! " << codec << " target-bitrate=" << target_bitrate <<
	" control-rate=variable ! video/x-h264, width=" << width << ",height=" << height << ",framerate=30/1,profile=high ! rtph264pay ! gdppay ! udpsink"
	<< " host=" << recieveAddress << " port=" << port;

	string strCommand = command.str();
	cout << "> " << strCommand << endl;
	
	pid_t pid = fork();
	
	if (pid == 0) {
		// child
		close(servFd);
		
		const char *argv[] = {
			"/bin/sh",
			"-c",
			strCommand.c_str(),
			nullptr
		};
		
		execv("/bin/sh", (char *const* )argv);
		exit(127);
	}
	else gstreamerPID = pid;
}

Streamer::Streamer(int width, int height) : width(width), height(height) {
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
		servAddr.sin6_port = htons(8080);
	
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

			if (gstreamerPID > 0) {
				cout << "killing previous instance: " << gstreamerPID << endl;
				if (kill(gstreamerPID, SIGTERM) == -1) {
					perror("kill");
				}
			}
			sleep(1);
			
			const char message[] = "Launching remote GStreamer...\n";
			if (write(clientFd, message, sizeof(message)) == -1) {
				perror("write");
			}

			if (close(clientFd) == -1) perror("close");

			// wait for client's gstreamer to initialize
			sleep(2);

			char strAddr[INET6_ADDRSTRLEN];
			inet_ntop(AF_INET6, &(clientAddr.sin6_addr), strAddr, sizeof(strAddr));
			launchGStreamer(strAddr);			
		}
	}).detach();
}






void Streamer::_writeFrame() {
	std::chrono::steady_clock clock;
	auto drawStartTime = clock.now();
	
	cv::Mat drawnOn = image.clone();
	
	for (auto i = toDraw.begin(); i < toDraw.end(); ++i) {
		
		cv::Scalar color;
		if (i == toDraw.begin()) color = cv::Scalar(0, 255, 0);
		else color = cv::Scalar(255, 0, 0);
		
		cv::rectangle(drawnOn, toDraw[0].right, color);
		cv::rectangle(drawnOn, toDraw[0].left, color);
	}
	
	auto drawEndTime = clock.now();
	cout << "drawing data took: " << std::chrono::duration_cast<std::chrono::milliseconds>
	(drawEndTime - drawStartTime).count() << " ms" << endl;
	
	fwrite(drawnOn.data, drawnOn.total() * drawnOn.elemSize(), 1, videoFifo);
	
	cout << "writing frame took: " << std::chrono::duration_cast<std::chrono::milliseconds>
	(clock.now() - drawEndTime).count() << " ms" << endl;
}

void Streamer::writeFrame(cv::Mat image, std::vector<VisionTarget>& toDraw) {
	this->image = image; this->toDraw = toDraw;
	
	waitLock.unlock();
	condition.notify_all();
}

void Streamer::run() {
	while (true) {
		std::unique_lock<std::mutex> uniqueLock(waitLock);
		condition.wait(uniqueLock);
		_writeFrame();
	}
}
