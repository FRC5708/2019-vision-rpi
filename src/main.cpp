#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <fstream>
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

#include "DataComm.h"

using std::cout; using std::cerr; using std::endl; using std::string;

namespace vision5708Main {

	FILE* videoFifo;
	
	cv::Mat currentFrame;
	std::vector<VisionTarget> lastResults;

	std::chrono::steady_clock clock;
	auto currentFrameTime = clock.now();
	
	std::mutex waitMutex;
	std::condition_variable condition;
	
	
	void VisionThread() {
		DataComm rioComm=DataComm("10.57.8.2");

		
		while (true) {
			auto lastFrameTime = currentFrameTime;
			lastResults = doVision(currentFrame);
			
			
			std::vector<VisionData> calcs;
			calcs.reserve(lastResults.size());
			for (auto i : lastResults) {
				calcs.push_back(i.calcs);
			} 
			
			rioComm.sendData(calcs, lastFrameTime);
			
			if (lastFrameTime == currentFrameTime) { // no new frame yet
				std::unique_lock<std::mutex> uniqueWaitMutex(waitMutex);
				condition.wait(uniqueWaitMutex);
			}
		}
	}

	void setDefaultCalibParams() {
		calib::width = 1280; calib::height = 720;
		
		//constexpr double radFOV = (69.0/180.0)*M_PI;
		constexpr double radFOV = (78.0/180.0)*M_PI;
		const double pixFocalLength = tan((M_PI_2) - radFOV/2) * sqrt(pow(calib::width, 2) + pow(calib::height, 2))/2; // pixels. Estimated from the camera's FOV spec.
		
		static double cameraMatrixVals[] {
			pixFocalLength, 0, ((double) calib::width)/2,
			0, pixFocalLength, ((double) calib::height)/2,
			0, 0, 1
		};
		calib::cameraMatrix = cv::Mat(3, 3, CV_64F, cameraMatrixVals);
		// distCoeffs is empty matrix
	}
	bool readCalibParams(const char* path, bool failHard = true) {
		cv::FileStorage calibFile;
		calibFile.open(path, cv::FileStorage::READ);
		if (!calibFile.isOpened()) {
			std::cerr << "Failed to open camera data " << path << endl;
			if (failHard) exit(1);
			else return false;
		}
		
		//setDefaultCalibParams();
		calib::cameraMatrix = calibFile["cameraMatrix"].mat();
		calib::distCoeffs = calibFile["dist_coeffs"].mat();
		cv::FileNode calibSize = calibFile["cameraResolution"];

		calib::width = calibSize[0];
		calib::height = calibSize[1];
		
		assert(calib::cameraMatrix.type() == CV_64F);
		
		// correcting for opencv bug?
		//calib::cameraMatrix.at<double>(0,0) *= 2;
		//calib::cameraMatrix.at<double>(1,1) *= 2;
		
		cout << "Loaded camera data: " << path << endl;
		return true;
	}
	// change camera calibration to match resolution of incoming image
	void changeCalibResolution(int width, int height) {
		assert(calib::cameraMatrix.type() == CV_64F);
		if (fabs(calib::width / (double) calib::height - width / (double) height) > 0.01) {
			cerr << "wrong aspect ratio recieved from camera" << endl;
			exit(1);
		}
		calib::cameraMatrix.at<double>(0, 0) *= (width / (double) calib::width);
		calib::cameraMatrix.at<double>(0, 2) *= (width / (double) calib::width);
		calib::cameraMatrix.at<double>(1, 1) *= (height / (double) calib::height);
		calib::cameraMatrix.at<double>(1, 2) *= (height / (double) calib::height);

		calib::width = width; calib::height = height;
		
		cout << "camera matrix set to: " << calib::cameraMatrix << endl;
	}
	void doImageTesting(const char* path) {
		isImageTesting = true; verboseMode = true;
				
		cv::Mat image=cv::imread(path);
		cout << "image size: " << image.cols << 'x' << image.rows << endl;
		changeCalibResolution(image.cols, image.rows);

		std::vector<VisionTarget> te = doVision(image);
		cout << "Testing Path: " << path << std::endl;
		for(auto &i:te){
			auto calc=i.calcs;
			cout << "Portland: " << calc.isPort << " Distance: " << calc.distance << " tape: " << calc.tapeAngle << " robot: " << calc.robotAngle << std::endl;
			cout << "L: " << i.left.x << ":" << i.left.y << " " << i.left.width << "," << i.left.height
			<< " R: " << i.right.x << ":" << i.right.y << " " << i.right.width << "," << i.right.height << std::endl;
		}
	}
	bool fileIsImage(char* file) {
		string path(file);
		string extension = path.substr(path.find_last_of(".") + 1);
		for (auto & c: extension) c = toupper(c);
		return extension == "PNG" || extension == "JPG" || extension == "JPEG";
	}
	
	//#define VERBOSE

	int main(int argc, char** argv) {

		if (argc >= 3) {
			readCalibParams(argv[1]);
			doImageTesting(argv[2]);
			return 0;
		}
		else if (argc == 2) {
			if (fileIsImage(argv[1])) {
				setDefaultCalibParams();
				doImageTesting(argv[1]);
				return 0;
			}
			else readCalibParams(argv[1]);
		}
		else if (argc == 1) {
			if (!readCalibParams("/home/pi/vision-code/calib_data/logitech_c920.xml", false)) {
				setDefaultCalibParams();
			}
		}
		else {
			cerr << "invalid number of arguments" << endl;
			return 1;
		}

		//#ifdef VERBOSE
		verboseMode = true;
		//#endif

		signal(SIGPIPE, SIG_IGN);

		Streamer streamer;
		streamer.launchFFmpeg();

		cv::VideoCapture camera;
		
		bool success = false;
		for (int i=0; !success; ++i) {
			
			
			#ifdef __linux__
			success = camera.open("/dev/video1");
			#else
			success = camera.open(1);
			#endif
			cout << "camera opening " << (success? ("succeeded @/dev/video1") : "failed") << endl;
			if (!success) usleep(200000); // 200 ms
		}
		if(!success){
			cout << "Camera opening unsuccessful" << endl; 
			return -1;
		}
		
		while (!camera.read(currentFrame)) {
			usleep(5000);
		}
		int imgWidth = currentFrame.cols, imgHeight = currentFrame.rows;
		cout << "Got first frame. width=" << imgWidth << ", height=" << imgHeight << endl;
		
		streamer.start(imgWidth, imgHeight);		
		
		changeCalibResolution(imgWidth, imgHeight);

		std::thread thread(&VisionThread);
		
		while (true) {
			#ifdef VERBOSE
			auto frameReadStart = clock.now();
			#endif

			camera.grab();

			#ifdef VERBOSE
			cout << "grabbing frame took: " << std::chrono::duration_cast<std::chrono::milliseconds>
			(clock.now() - frameReadStart).count() << " ms" << endl;
			frameReadStart = clock.now();
			#endif

			camera.retrieve(currentFrame);

			#ifdef VERBOSE
			cout << "retrieving frame took: " << std::chrono::duration_cast<std::chrono::milliseconds>
			(clock.now() - frameReadStart).count() << " ms" << endl;
			#endif

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
