#include <opencv2/opencv.hpp>
#include <math.h>
#include <unistd.h>
#include <string>
#include <chrono>


using std::cout; using std::endl;

class Encoder {
public:
	FILE* videoFifo;

	Encoder(int width, int height) {
		

		std::string filesDir = std::string(getenv("HOME")) + "/vision_files";
		std::string streamPath = filesDir + "/video_stream";
		
		system(("mkfifo " + streamPath).c_str());

		#ifdef __linux__
		std::string codec = "h264_omx";
		#else
		std::string codec = "h264_videotoolbox -realtime";
		#endif

		std::string recieveAddress = "10.126.58.248";

		std::stringstream command;
		command << "ffmpeg -re -f rawvideo -pixel_format bgr24 -video_size "
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
	void encode_frame(cv::Mat image) {
		fwrite(image.data, image.total() * image.elemSize(), 1, videoFifo);
	}
};

int main(int argc, char** argv) {
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
	
	cv::Mat image;

	while (!camera.read(image)) {
			usleep(5000);
	}
	cout << "Got first frame. width=" << image.cols << ", height=" << image.rows << endl;
	Encoder encoder(image.cols, image.rows);
	cout << "Initialized video streamer" << endl;

	/*
	std::chrono::steady_clock clock;
	auto beginning = clock.now();
	auto lastFrame = clock.now();
	int frameCount = 0;
	*/
	while (true) {
		while (!camera.read(image)) {
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
		encoder.encode_frame(image);  
	}
}

