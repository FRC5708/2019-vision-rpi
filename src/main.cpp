#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <math.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>

#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>

#include "vision.hpp"


using std::cout; using std::endl; using std::string;

namespace vision5708Main {

	FILE* videoFifo;
	
	cv::Mat currentFrame;
	std::vector<VisionTarget> lastResults;

	void initializeStreaming(int width, int height) {
		
		string filesDir = string(getenv("HOME")) + "/vision_files";
		string streamPath = filesDir + "/video_stream";
		
		system(("mkfifo " + streamPath).c_str());
		
		#ifdef __linux__
		string codec = "h264_omx";
		string ffmpegCommand = "ffmpeg";
		#elif defined __APPLE__
		string codec = "h264_videotoolbox";
		string ffmpegCommand = "/usr/local/bin/ffmpeg";
		#endif
		
		string recieveAddress = "127.0.0.1";
		
		std::stringstream command;
		command << ffmpegCommand << " -re -f rawvideo -pixel_format bgr24 -video_size "
		<< width << "x" << height
		<< " -r 60 -i " << streamPath << " -c:v " << codec
		<< " -b:v 3000k -sdp_file " << filesDir << "/stream.sdp"
		<< " -f rtp rtp://" << recieveAddress << ":5004 &";
		
		cout << command.str() << endl;
		system(command.str().c_str());
		//system("ffmpeg -re -f rawvideo -pixel_format bgr24 -video_size 1280x720 -i ~/video_stream.bgr24 -c:v h264_videotoolbox -b:v 3000k -sdp_file ~/test.sdp -f rtp rtp://localhost:5004/  &");
		
		
		videoFifo = fopen(streamPath.c_str(), "a");
		// play stream with:
		// ffplay -protocol_whitelist "file,rtp,udp" -fflags nobuffer -flags low_delay -framedrop -strict -experimental -analyzeduration 1 -sync ext -i path_to_sdp_file
		// It needs to be started BEFORE this program for stupid reasons
	}
	void writeFrame(cv::Mat image) {
		cv::Mat drawnOn = image.clone();
		
		for (auto i = lastResults.begin(); i < lastResults.end(); ++i) {
			
			cv::Scalar color;
			if (i == lastResults.begin()) color = cv::Scalar(0, 255, 0);
			else color = cv::Scalar(255, 0, 0);
			
			cv::rectangle(drawnOn, lastResults[0].right, color);
			cv::rectangle(drawnOn, lastResults[0].left, color);
		}
		
		fwrite(drawnOn.data, drawnOn.total() * drawnOn.elemSize(), 1, videoFifo);
	}
	
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
			
			for (int i = 0; i != data.size(); ++i) {
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
		
		cv::VideoCapture camera;
		
		bool success = false;
		while (!success) {
			#ifdef __linux__
			success = camera.open("/dev/video0");
			#else
			success = camera.open(0);
			#endif
			cout << "camera opening " << (success? "succeeded" : "failed") << endl;
			if (!success) usleep(200000); // 200 ms
		}
		
		while (!camera.read(currentFrame)) {
			usleep(5000);
		}
		cout << "Got first frame. width=" << currentFrame.cols << ", height=" << currentFrame.rows << endl;
		initializeStreaming(currentFrame.cols, currentFrame.rows);
		cout << "Initialized video streamer" << endl;
		
		std::thread thread(&VisionThread);
		
		while (true) {
			while (!camera.read(currentFrame)) {
				usleep(5000);
			}
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
			
			writeFrame(currentFrame);
		}
	}
}
int main(int argc, char** argv) {
	return vision5708Main::main(argc, argv);
}
