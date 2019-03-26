#include "streamer.hpp"
#include "DataComm.hpp"

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


using std::cout; using std::endl; using std::string;

pid_t runCommandAsync(const std::string& cmd, int closeFd) {
	pid_t pid = fork();
	
	if (pid == 0) {
		// child
		close(closeFd);

		string execCmd = ("exec " + cmd);
		cout << "> " << execCmd << endl;
		
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

void Streamer::handleCrash(pid_t pid) {
	if (!handlingLaunchRequest) {
		for (auto i : gstInstances) {
			if (i.pid == pid) {
				runCommandAsync(i.command, servFd);
			}
		}
	}
}

void Streamer::launchGStreamer(const char* recieveAddress, int bitrate, string port, string file) {
	cout << "launching GStreamer, targeting " << recieveAddress << endl;
	
	string codec = "omxh264enc";
	string gstreamCommand = "gst-launch-1.0";
	
	std::stringstream command;
	command << gstreamCommand << "v4l2src device=" << file << " ! videoscale ! videoconvert ! queue ! " << codec << " target-bitrate=" << bitrate <<
	" control-rate=variable ! video/x-h264, width=" << width << ",height=" << height << ",framerate=30/1,profile=high ! rtph264pay ! gdppay ! udpsink"
	<< " host=" << recieveAddress << " port=" << port;

	string strCommand = command.str();
	
	pid_t pid = runCommandAsync(strCommand, servFd);

	gstInstances.push_back({ pid, file, strCommand });
}

void Streamer::launchFFmpeg() {
	ffmpegPID = runCommandAsync(
		"ffmpeg -f v4l2 -pix_fmt yuyv422 -video_size  800x448 -i /dev/video0 -f v4l2 /dev/video1 -f v4l2 /dev/video2"
	, servFd);
}


string getVideoDeviceWithString(string cmp) {

	FILE* videos = popen(("for I in /sys/class/video4linux/*; do if grep -q '" 
	+ cmp + "' $I/name; then basename $I; exit; fi; done").c_str(), "r");

	char output[1035];
	string devname;
	while (fgets(output, sizeof(output), videos) != NULL) {
		// videoX
		if (strlen(output) >= 6) devname = output;
		// remove newlines
		devname.erase(std::remove(devname.begin(), devname.end(), '\n'), devname.end());
	}
	pclose(videos);
	if (!devname.empty()) return "/dev/" + devname;
	else return "";
}

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

			for (auto i : gstInstances) {
				
				cout << "killing previous instance: " << i.pid << "   " << endl;
				if (kill(i.pid, SIGTERM) == -1) {
					perror("kill");
				}
				waitpid(i.pid, nullptr, 0);
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

			launchGStreamer(strAddr, atoi(bitrate), "5809", loopbackDev);
			if (!secondCameraDev.empty()) launchGStreamer(strAddr, atoi(bitrate), "5805", secondCameraDev);

			cout << "Starting UDP stream..." << endl;
			if (computer_udp) delete computer_udp;
			computer_udp = new DataComm(strAddr, "5806");

			handlingLaunchRequest = false;
		}
	}).detach();

	// TODO
	visionCameraDev = getVideoDeviceWithString("920");
	secondCameraDev = getVideoDeviceWithString("C525");
	loopbackDev = getVideoDeviceWithString("Dummy");

	if (visionCameraDev.empty()) {
		if (!secondCameraDev.empty()) {
			visionCameraDev = secondCameraDev;
			secondCameraDev = "";
		}
		else {
			std::cerr << "Camera not found" << std::endl;
			exit(1);
		}
	}
	std::cout << "main camera: " << visionCameraDev << std::endl;

	if (secondCameraDev.empty()) {
		std::cerr << "Warning: second camera not found" << std::endl;
	}
	else std::cout << "second camera: " << visionCameraDev << std::endl;
	if (loopbackDev.empty()) {
		std::cerr << "v4l2loopback device not found" << std::endl;
		exit(1);
	}
	else std::cout << "video loopback device: " << visionCameraDev << std::endl;

	camera.openReader(width, height, visionCameraDev.c_str());
	videoWriter.openWriter(width, height, loopbackDev.c_str());
}

void Streamer::setDrawTargets(std::vector<VisionTarget>* drawTargets) {
	this->drawTargets = drawTargets;
	//if (computer_udp) computer_udp->sendDraw(&(*drawTargets)[0].drawPoints);
}

cv::Mat Streamer::getBGRFrame() {
	// get frame for vision processing
	cv::Mat frame;
	cvtColor(camera.getMat(), frame, cv::COLOR_YUV2BGR_YUYV);
	return frame;
}

void Streamer::run(std::function<void(void)> frameNotifier) {
	
	//std::chrono::steady_clock clock;
	while (true) {
		//auto startTime = clock.now();
		camera.grabFrame();
		//auto writeStart = clock.now();
		//std::cout << "grabFrame took: " << std::chrono::duration_cast<std::chrono::milliseconds>
		//	(writeStart - startTime).count() << " ms" << endl;

		cv::Mat drawnOn = camera.getMat().clone();

		if (drawTargets != nullptr) {
			for (auto i = drawTargets->begin(); i < drawTargets->end(); ++i) {
				drawVisionPoints(i->drawPoints, drawnOn);
			}
		}

		videoWriter.writeFrame(drawnOn);

		//std::cout << "drawing and writing took: " << std::chrono::duration_cast<std::chrono::milliseconds>
		//	(clock.now() - writeStart).count() << " ms" << endl;

		frameNotifier();
	}
	
}
