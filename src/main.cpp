#include <opencv2/opencv.hpp>
#include <math.h>
#include <unistd.h>
#include <string>
#include <chrono>
extern "C" {
	#include <libavcodec/avcodec.h>
	#include <libavutil/common.h>
	#include <libavutil/imgutils.h>
	#include <libavutil/opt.h>
}

using std::cout; using std::endl;

/*
For streaming:
--Use mkfifo
then use VLC to multicast a RTP stream to 239.0.0.1 (https://wiki.videolan.org/Documentation:Streaming_HowTo/Advanced_Streaming_Using_the_Command_Line/#rtp)
then use VLC on driver console to connect to that stream--

Actually, just use GNU ccRTP
*/

//#define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : X)
// RGB -> YUV
//#define RGB2Y(R, G, B) CLIP(( (  66 * (R) + 129 * (G) +  25 * (B) + 128) >> 8) +  16)
//#define RGB2U(R, G, B) CLIP(( ( -38 * (R) -  74 * (G) + 112 * (B) + 128) >> 8) + 128)
//#define RGB2V(R, G, B) CLIP(( ( 112 * (R) -  94 * (G) -  18 * (B) + 128) >> 8) + 128)

#define RGB2Y(R, G, B) (0.257 * R) + (0.504 * G) + (0.098 * B) + 16
#define RGB2U(R, G, B) -(0.148 * R) - (0.291 * G) + (0.439 * B) + 128
#define RGB2V(R, G, B) (0.439 * R) - (0.368 * G) - (0.071 * B) + 128

AVCodec* thing;
class Encoder {
public:
	 AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
	AVCodecContext *ctx;
	AVFrame *frame = av_frame_alloc();
	AVPacket pkt;

	FILE* videoFifo;

	Encoder(int width, int height) {
		ctx = avcodec_alloc_context3(codec);
		ctx->bit_rate = 3000000; // 3 megabits 
		ctx->bit_rate_tolerance = 500000; // 500 kilobits

		ctx->width = width;
		ctx->height = height;
		ctx->time_base = (AVRational){1,30};
		ctx->pix_fmt = AV_PIX_FMT_YUV420P;

		ctx->gop_size = 10; /* emit one intra frame every ten frames */
		ctx->max_b_frames = 1;
		ctx->global_quality = 1;

		//av_opt_set(ctx->priv_data, "preset", "slow", 0);

		avcodec_open2(ctx, codec, nullptr);

		frame->format = ctx->pix_fmt;
	 	frame->width  = ctx->width;
		frame->height = ctx->height;

		av_frame_get_buffer(frame, 0);

		system("mkfifo ~/video_stream.h264");

		#ifdef __APPLE__
		std::string vlcCommand = "/Applications/VLC.app/Contents/MacOS/VLC";
		#else
		std::string vlcCommand = "vlc";
		#endif

		//system((vlcCommand + 
		//" ~/video_stream.h264 --intf dummy --sout \"#rtp{dst=127.0.0.1,port=5004,sdp=http://:5006/}\" -demux h264 &").c_str());

		std::string home = getenv("HOME");
		videoFifo = fopen((home + "/video_stream.h264").c_str(), "a");
	}
	void encode_frame(cv::Mat image) {
		av_init_packet(&pkt);
		pkt.data = NULL;
		pkt.size = 0;

		//frame->data[0] = image.data; // Might need some sort of conversion
		//av_image_alloc(frame->data, frame->linesize, ctx->width, ctx->height,
		//				 ctx->pix_fmt, 32);
		for (int y = 0; y < ctx->height; y++) {
			for (int x = 0; x < ctx->width; x++) {
				cv::Vec3b px = image.at<cv::Vec3b>(y, x);
				frame->data[0][y * frame->linesize[0] + x] = RGB2Y(px[0], px[1], px[2]);
			}
		}

		// Cb and Cr 
		for (int y = 0; y < ctx->height/2; y++) {
			for (int x = 0; x < ctx->width/2; x++) {
				cv::Vec3b px = (image.at<cv::Vec3b>(y, x) + image.at<cv::Vec3b>(x+1, y)
				 + image.at<cv::Vec3b>(x, y+1) + image.at<cv::Vec3b>(x+1, y+1)) / 4;
				frame->data[1][y * frame->linesize[1] + x] =  RGB2U(px[0], px[1], px[2]);
				frame->data[2][y * frame->linesize[2] + x] =  RGB2V(px[0], px[1], px[2]);
			}
		}
		if (avcodec_send_frame(ctx, frame) < 0) {
			fprintf(stderr, "Error encoding frame\n");
			exit(1);
		}
			
		while (true) {
			avcodec_receive_packet(ctx, &pkt);
			if (pkt.size > 0) {
				fwrite(pkt.data, pkt.size, 1, videoFifo);
				cout << "wrote packet of size " << pkt.size << endl;
				av_packet_unref(&pkt); 
			}
			else break;
		}
		//av_frame_unref(frame);
	}
};

int main(int argc, char** argv) {
	cv::VideoCapture camera;

	bool success = false;
		 while (!success) {
			success = camera.open(0);
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

	std::chrono::steady_clock clock;
	auto beginning = clock.now();
	auto lastFrame = clock.now();
	int frameCount = 0;

	while (true) {
		while (!camera.read(image)) {
			usleep(5000);
		}	
		frameCount++;
		cout << "encoding frame. Instant FPS: " << 
		1.0/std::chrono::duration<double>(clock.now() - lastFrame).count()
		 << "; Average FPS: " << 
		frameCount * (1.0/std::chrono::duration<double>(clock.now() - beginning).count()) << endl;

		lastFrame = clock.now();

		encoder.encode_frame(image);  
	}
}

