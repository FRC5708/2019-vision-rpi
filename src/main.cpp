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
#include <arpa/inet.h>

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
		int fd;
		sockaddr_in clientAddr;
		char* client_ip;
		RioComm(char* ip){
			client_ip=ip;
		}
	public:
		RioComm() {
			if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
				perror("socket failed");
			}
			int opt=1;
			if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
				perror("setsockopt failed");
			}
			clientAddr.sin_family=AF_INET;
			int ret = inet_aton(client_ip, &clientAddr.sin_addr);
  			if (ret == 0) { 
				  perror("inet_aton");
			}
			clientAddr.sin_port=htons(8081);
			
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
			
			sendto(fd, &sendStr, sendStr.length, 0, (sockaddr*)&clientAddr, sizeof(clientAddr));
			cout << sendStr;
		}
	};
	
	void VisionThread() {
		string addr="127.0.0.1";
		RioComm rioComm=RioComm(addr.c_str);
		
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
		system("ffmpeg -f v4l2 -i /dev/video0 -f v4l2 /dev/video1 -f v4l2 /dev/video2 > /dev/null 2>&1 &");
		
		cv::VideoCapture camera;
		
		bool success = false;
		for (int cameraId = 1; !success; ++cameraId) {
			
			if (cameraId > 5) cameraId = 0;
			
			#ifdef __linux__
			success = camera.open("/dev/video" + std::to_string(cameraId));
			#else
			success = camera.open(cameraId);
			#endif
			cout << "camera opening " << (success? ("succeeded @/dev/video" + cameraId) : "failed") << endl;
			if (!success) usleep(200000); // 200 ms
		}
		
		camera.set(cv::CAP_PROP_FRAME_WIDTH, 853);
		camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
		
		while (!camera.read(currentFrame)) {
			usleep(5000);
		}
		cout << "Got first frame. width=" << currentFrame.cols << ", height=" << currentFrame.rows << endl;
		
		Streamer streamer(currentFrame.cols, currentFrame.rows);
		cout << "Initialized video streamer" << endl;
		
		std::thread thread(&VisionThread);
		
		
		while (true) {
			auto frameReadStart = clock.now();
			while (!camera.read(currentFrame)) {
				usleep(5000);
			}
			cout << "reading frame took: " << std::chrono::duration_cast<std::chrono::milliseconds>
			(clock.now() - frameReadStart).count() << " ms" << endl;
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
			
			streamer.writeFrame(currentFrame, lastResults);
		}
	}
}
int main(int argc, char** argv) {
	return vision5708Main::main(argc, argv);
}
