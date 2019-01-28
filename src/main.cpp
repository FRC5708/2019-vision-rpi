#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string>
#include <chrono>
#include <thread>

#include <mutex>
#include <condition_variable>

#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>

#include <signal.h>

#include "vision.hpp"
#include "VideoHandler.hpp"


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
		int fd;
		sockaddr_in clientAddr;
		
	public:
		RioComm() {
			
			if ((fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
				perror("socket failed");
			}
			
			int opt = 1;
			if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
				perror("setsockopt failed");
			}
			
			sockaddr_in servAddr;
			servAddr.sin_family = AF_INET;
			servAddr.sin_addr.s_addr = INADDR_ANY;
			servAddr.sin_port = htons(8081);
			
			if (bind(fd, (sockaddr*) &servAddr, sizeof(servAddr)) < 0) {
				perror("bind failed");
			}
			fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
			
			if (listen(fd, 5) < 0) perror("listen failed");
			
		}
		
		void sendData(std::vector<VisionData> data, std::chrono::time_point<std::chrono::steady_clock> timeFrom) {
			
			std::stringstream toSend;
			
			toSend << "@" <<
			std::chrono::duration_cast<std::chrono::milliseconds>(clock.now() - timeFrom).count()
			<< endl;
			
			for (unsigned int i = 0; i != data.size(); ++i) {
				char buf[200];
				sprintf(buf, "#%d: isPort=%d distance=%f tapeAngle=%f robotAngle=%f\n",
						i, data[i].isPort, data[i].distance, data[i].tapeAngle, data[i].robotAngle);
				toSend << buf;
			}
			
			string sendStr = toSend.str();
			
			if (write(fd, sendStr.c_str(), sendStr.length()) == -1) {
				// poll the connection
				unsigned int clientAddrLen = sizeof(clientAddr);
				int newfd = accept(fd, (sockaddr*) &clientAddr, &clientAddrLen);
				
				if (newfd < 0) perror("Rio not connected");
				else fd = newfd;
			}
			cout << sendStr;
		}
	};
	
	void VisionThread() {
		RioComm rioComm;
		
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
		signal(SIGPIPE, SIG_IGN);

		VideoHandler videoHandler(1280, 720);
		
		std::thread thread(&VisionThread);
		
		
		while (true) {
			currentFrame = videoHandler.readFrame();
			/*
			 frameCount++;
			 cout << "encoding frame. Instant FPS: " <<
			 1.0/std::chrono::duration<double>(clock.now() - lastFrame).count()
			 << "; Average FPS: " <<
			 frameCount * (1.0/std::chrono::duration<double>(clock.now() - beginning).count()) << endl;
			 
			 lastFrame = clock.now();
			 */
			currentFrameTime = clock.now();
			waitMutex.unlock();
			condition.notify_one();
			
			videoHandler.writeFrame(videoHandler.drawFrame(currentFrame, lastResults));
		}
	}
}
int main(int argc, char** argv) {
	return vision5708Main::main(argc, argv);
}
