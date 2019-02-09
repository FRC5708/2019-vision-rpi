#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string>
#include <chrono>
#include <thread>

#include <mutex>
#include <condition_variable>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>

#include <signal.h>

#include "vision.hpp"
#include "streamer.hpp"

using std::cout; using std::endl; using std::string;

namespace vision5708Main {

	FILE* videoFifo;
	
	cv::Mat currentFrame;
	std::vector<VisionTarget> lastResults;

	std::chrono::steady_clock clock;
	auto currentFrameTime = clock.now();
	
	std::mutex waitMutex;
	std::condition_variable condition;
	
	class RioComm {
		int fd = -1;
		const char* client_name;
		
	public:
		void setupSocket() {
			fd = -1;
			
			// addrinfo is linked list
			struct addrinfo* addrs;
			
			struct addrinfo hints;
			memset(&hints, 0, sizeof(hints));
			hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
			hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
			
			int error = getaddrinfo(client_name, "5800", &hints, &addrs);
			if (error != 0) {
				fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(error));
			}
			
			for (struct addrinfo* rp = addrs; rp != nullptr; rp = rp->ai_next) {
				
				fd = socket(rp->ai_family, rp->ai_socktype,
							 rp->ai_protocol);
				if (fd == -1) continue;
				
				if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) {
					break;
				}
				else {
					close(fd);
					fd = -1;
					perror("connect failed");
				}
			}
			freeaddrinfo(addrs);
			if (fd == -1) {
				printf("could not resolve or connect to: %s\n", client_name);
				return;
			}
		}
		RioComm(const char* client_name) : client_name(client_name) {
			setupSocket();
		}
		
		void sendData(std::vector<VisionData> data, std::chrono::time_point<std::chrono::steady_clock> timeFrom) {
			std::stringstream toSend;
			
			if (fd < 0) setupSocket();
			
			if (fd >= 0) {
			
				for (unsigned int i = 0; i != data.size(); ++i) {
					char buf[200];
					sprintf(buf, "#%d: isPort=%d distance=%f tapeAngle=%f robotAngle=%f\n",
							i, data[i].isPort, data[i].distance, data[i].tapeAngle, data[i].robotAngle);
					toSend << buf;
				}
				toSend << "@" <<
				std::chrono::duration_cast<std::chrono::milliseconds>(clock.now() - timeFrom).count()
				<< endl;
				
				string sendStr = toSend.str();
				
				if (send(fd, sendStr.c_str(), sendStr.length(), 0) < 0) {
					perror("Failed to send data");
					setupSocket();
				}
				cout << sendStr;
			}
		}
	};
	
	void VisionThread() {
		RioComm rioComm=RioComm("10.57.8.2");
		
		while (true) {
			auto lastFrameTime = currentFrameTime;
			lastResults = doVision(currentFrame);
			
			
			std::vector<VisionData> calcs(lastResults.size());
			for (VisionTarget i : lastResults) calcs.push_back(i.calcs);
			
			rioComm.sendData(calcs, lastFrameTime);
			
			if (lastFrameTime == currentFrameTime) { // no new frame yet
				std::unique_lock<std::mutex> uniqueWaitMutex(waitMutex);
				condition.wait(uniqueWaitMutex);
			}
		}
	}
	
	int main(int argc, char** argv) {
		if (argc > 1) {
			isImageTesting = true;
			
			cv::Mat image=cv::imread(argv[1]);
			cout << "image size: " << image.cols << 'x' << image.rows << endl;
			std::vector<VisionTarget> te = doVision(image);
			cout << "Testing Path: " << argv[1] << std::endl;
			for(auto &i:te){
				auto calc=i.calcs;
				cout << "Portland: " << calc.isPort << " Distance: " << calc.distance << " tape: " << calc.tapeAngle << " robot: " << calc.robotAngle << std::endl;
				cout << "L: " << i.left.x << ":" << i.left.y << " " << i.left.width << "," << i.left.height
				 << " R: " << i.right.x << ":" << i.right.y << " " << i.right.width << "," << i.right.height << std::endl;
			}
		return 0;
		}

		signal(SIGPIPE, SIG_IGN);
		system("ffmpeg -f v4l2 -pix_fmt yuyv422 -i /dev/video0 -f v4l2 /dev/video1 -f v4l2 /dev/video2 &");
		
		cv::VideoCapture camera;
		
		bool success = false;
		for (int cameraId = 1; !success; ++cameraId) {
			
			if (cameraId > 5) cameraId = 0;
			
			#ifdef __linux__
			success = camera.open("/dev/video" + std::to_string(cameraId));
			cout << "camera opening " << (success? ("succeeded @/dev/video" + std::to_string(cameraId)) : "failed") << endl;
			#else
			success = camera.open(cameraId - 1);
			cout << "camera opening " << (success? "succeeded" : "failed") << endl;
			#endif
			if (!success) usleep(200000); // 200 ms
		}
		
		while (!camera.read(currentFrame)) {
			usleep(5000);
		}
		cout << "Got first frame. width=" << currentFrame.cols << ", height=" << currentFrame.rows << endl;
		
		Streamer streamer(currentFrame.cols, currentFrame.rows);
		
		std::thread thread(&VisionThread);
		
		
		while (true) {
			auto frameReadStart = clock.now();
			camera.grab();
			cout << "grabbing frame took: " << std::chrono::duration_cast<std::chrono::milliseconds>
			(clock.now() - frameReadStart).count() << " ms" << endl;
			frameReadStart = clock.now();
			camera.retrieve(currentFrame);
			cout << "retrieving frame took: " << std::chrono::duration_cast<std::chrono::milliseconds>
			(clock.now() - frameReadStart).count() << " ms" << endl;
			
			
			currentFrameTime = clock.now();
			waitMutex.unlock();
			condition.notify_one();
			
			streamer.writeFrame(currentFrame, lastResults);
		}
	}
}
int main(int argc, char** argv) {
	return vision5708Main::main(argc, argv);
}
