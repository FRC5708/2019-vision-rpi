#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

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
//#include "streamer.hpp"

#include <fstream>

using std::cout; using std::endl; using std::string;
char* path;
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
			//cout << sendStr;
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
		/*
		signal(SIGPIPE, SIG_IGN);
		
		cv::VideoCapture camera;
		
		bool success = false;
		for (int cameraId = 0; !success; ++cameraId) {
			
			if (cameraId > 5) cameraId = 0;
			
			#ifdef __linux__
			success = camera.open("/dev/video" + std::to_string(cameraId));
			#else
			success = camera.open(cameraId);
			#endif
			cout << "camera opening " << (success? "succeeded" : "failed") << endl;
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
		
		//std::thread thread(&VisionThread);
		
		
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
			 *//*
			currentFrameTime = clock.now();
			waitMutex.unlock();
			condition.notify_one();
			
			streamer.writeFrame(currentFrame, lastResults);
			}*/

		cv::Mat image=cv::imread(path);
		cout << "image size: " << image.cols << 'x' << image.rows << endl;
		std::vector<VisionTarget> te = doVision(image);
		cout << "Testing Path: " << path << std::endl;
		for(auto &i:te){
			auto calc=i.calcs;
			cout << "Portland: " << calc.isPort << " Distance: " << calc.distance << " tape: " << calc.tapeAngle << " robot: " << calc.robotAngle << std::endl;
			cout << "L: " << i.left.x << ":" << i.left.y << " " << i.left.width << "," << i.left.height
			 << " R: " << i.right.x << ":" << i.right.y << " " << i.right.width << "," << i.right.height << std::endl;
		}
		return 0;
	}

}

int main(int argc, char** argv) {
	if(argc < 2){
		return -1;
	}
	path=argv[1];
	return vision5708Main::main(argc, argv);
}
