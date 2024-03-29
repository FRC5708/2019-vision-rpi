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
#include <pthread.h>

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

#include "DataComm.hpp"

using std::cout; using std::cerr; using std::endl; using std::string;

namespace vision5708Main {

	// when false, drastically slows down vision processing
	volatile bool visionEnabled = false;
	
	std::vector<VisionTarget> lastResults;

	std::chrono::steady_clock clock;
	auto currentFrameTime = clock.now();
	
	std::mutex waitMutex; 
	std::condition_variable condition;

	Streamer streamer;
	
	// recieves enable/disable signals from the RIO to conserve thermal capacity
	// Also sets exposure when actively driving to target
	void ControlSocket() {
		struct addrinfo hints;
		memset(&hints, 0, sizeof(hints));
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
		hints.ai_flags = AI_PASSIVE;   /* For wildcard IP address */

		struct addrinfo *result;

		int error = getaddrinfo(nullptr, "5805", &hints, &result);
		if (error != 0) {
			fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(error));
		}

		int sockfd;
		for (struct addrinfo *rp = result; rp != NULL; rp = rp->ai_next) {
				sockfd = socket(rp->ai_family, rp->ai_socktype,
						rp->ai_protocol);
			if (sockfd != -1) {

				if (bind(sockfd, rp->ai_addr, rp->ai_addrlen) == 0) break;
				else {
					perror("failed to bind to control socket");
					close(sockfd);
					sockfd = -1;
				}
			}
		}
		freeaddrinfo(result);

		if (sockfd == -1) {
			std::cout << "could not connect to control socket" << std::endl;
		}

		while (true) {
			char buf[66537];
			ssize_t recieveSize = recvfrom(sockfd, buf, sizeof(buf) - 1, 
			0, nullptr, nullptr);
			if (recieveSize > 0) {
				buf[recieveSize] = '\0';
				cout << buf << endl;
				string msgStr(buf);
				if (msgStr.find("ENABLE") != string::npos) visionEnabled = true;
				if (msgStr.find("DISABLE") != string::npos) visionEnabled = false;
				if (msgStr.find("DRIVEON") != string::npos) streamer.setLowExposure(true);
				if (msgStr.find("DRIVEOFF") != string::npos) streamer.setLowExposure(false);

			}
			else if (recieveSize < 0) {
				perror("control data recieve error");
			}
			else std::cerr << "empty packet??" << std::endl;
		}
	}

	void VisionThread() {
		// give this thread a lower priority
		errno = 0;
		nice(5);
		if (errno != 0) perror("nice");

		DataComm rioComm=DataComm("10.57.8.2", "5808");

		
		while (true) {
			// currentFrameTime serves as a unique marker for this frame
			auto lastFrameTime = currentFrameTime;
			lastResults = doVision(streamer.getBGRFrame());

			streamer.setDrawTargets(&lastResults);
			
			std::vector<VisionData> calcs;
			calcs.reserve(lastResults.size());
			for (auto i : lastResults) {
				calcs.push_back(i.calcs);
			} 
			
			rioComm.sendData(calcs, lastFrameTime);
			
			// If no new frame has come from the camera, wait.
			if (lastFrameTime == currentFrameTime) {
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

	void chldHandler(int sig, siginfo_t *info, void *ucontext) {
		streamer.handleCrash(info->si_pid);
	}

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
			if (!readCalibParams("/home/pi/Documents/2019-vision-rpi/calib_data/logitech_c920.xml", false)) {
				setDefaultCalibParams();
			}
		}
		else {
			cerr << "usage: " << argv[0] << "[test image][calibration parameters]" << endl;
			return 1;
		}
        
        system("/home/pi/bin/run_setup_v4l2loopback");
		verboseMode = true;

		
		// SIGPIPE is sent to the program whenever a connection terminates. We want the program to stay alive if a connection unexpectedly terminates.
		signal(SIGPIPE, SIG_IGN);

		streamer.start();

		// SIGCHLD is recieved if gStreamer unexpectedly terminates.
		struct sigaction sa;
		sigemptyset(&sa.sa_mask);
		sa.sa_flags = SA_RESTART | SA_NOCLDSTOP | SA_SIGINFO;
		sa.sa_sigaction = &chldHandler;
		if (sigaction(SIGCHLD, &sa, 0) == -1) {
			perror("sigaction");
			exit(1);
		}
		
		// Scale the calibration parameters to match the current resolution
		changeCalibResolution(streamer.width, streamer.height);

		std::thread visThread(&VisionThread);
		std::thread controlSockThread(&ControlSocket);

		// never returns
		streamer.run([]() {
			
			// This lambda is called every frame.
			// It wakes up the vision processing thread if it is waiting on a new frame.
			// if vision processing is disabled, it wakes up the thread only once every 2 seconds.
			
			auto time = clock.now();
			if (visionEnabled || (time - currentFrameTime) > std::chrono::seconds(2)) {

				currentFrameTime = time;
				waitMutex.unlock();
				condition.notify_one();
			}
		});

		return 0;
	}
	
}
int main(int argc, char** argv) {
	return vision5708Main::main(argc, argv);
}
