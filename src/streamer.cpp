#include "streamer.hpp"

#include <unistd.h>
#include <string>
#include <iostream>

#include <opencv2/imgproc.hpp>

using std::cout; using std::endl; using std::string;


	
	Streamer::Streamer(int width, int height) {
		
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
		
		string recieveAddress = "10.57.8.83";
		
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
	void Streamer::writeFrame(cv::Mat image, std::vector<VisionTarget>& toDraw) {
		cv::Mat drawnOn = image.clone();
		
		for (auto i = toDraw.begin(); i < toDraw.end(); ++i) {
			
			cv::Scalar color;
			if (i == toDraw.begin()) color = cv::Scalar(0, 255, 0);
			else color = cv::Scalar(255, 0, 0);
			
			cv::rectangle(drawnOn, toDraw[0].right, color);
			cv::rectangle(drawnOn, toDraw[0].left, color);
		}
		
		fwrite(drawnOn.data, drawnOn.total() * drawnOn.elemSize(), 1, videoFifo);
	}

