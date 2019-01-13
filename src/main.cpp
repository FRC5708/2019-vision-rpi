#include <opencv2/opencv.hpp>
#include <math.h>
#include <unistd.h>
extern "C" {
	#include <libavcodec/avcodec.h>
	#include <libavutil/common.h>
	#include <libavutil/imgutils.h>
}

using cout, endl;

/*
For streaming:
--Use mkfifo
then use VLC to multicast a RTP stream to 239.0.0.1 (https://wiki.videolan.org/Documentation:Streaming_HowTo/Advanced_Streaming_Using_the_Command_Line/#rtp)
then use VLC on driver console to connect to that stream--

Actually, just use GNU ccRTP
*/
#define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : X)

// RGB -> YUV
#define RGB2Y(R, G, B) CLIP(( (  66 * (R) + 129 * (G) +  25 * (B) + 128) >> 8) +  16)
#define RGB2U(R, G, B) CLIP(( ( -38 * (R) -  74 * (G) + 112 * (B) + 128) >> 8) + 128)
#define RGB2V(R, G, B) CLIP(( ( 112 * (R) -  94 * (G) -  18 * (B) + 128) >> 8) + 128)

AVCodec* thing;
class Encoder {
public:
	 AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
	AVCodecContext *c;
	AVFrame *frame = av_frame_alloc();
	AVPacket pkt;

	Encoder(int width, int height) {
		c = avcodec_alloc_context3(codec);
		c->bit_rate = 3000000; // 3 megabits 

		c->width = width;
		c->height = height;
		c->time_base = (AVRational){1,30};
		c->pix_fmt = AV_PIX_FMT_YUV420P;

		c->gop_size = 10; /* emit one intra frame every ten frames */
		c->max_b_frames = 1;

		avcodec_open2(c, codec, nullptr);
		cout << "created codec" << endl;

		frame->format = c->pix_fmt;
	 	frame->width  = c->width;
		frame->height = c->height;


	}
	void encode_frame(cv::Mat image) {
		int got_output;

		av_init_packet(&pkt);
		pkt.data = NULL;
		pkt.size = 0;

		//frame->data[0] = image.data; // Might need some sort of conversion
		av_image_alloc(frame->data, frame->linesize, c->width, c->height,
						 c->pix_fmt, 32);
		for (int y = 0; y < c->height; y++) {
			for (int x = 0; x < c->width; x++) {
				cv::Vec3b px = image.at<cv::Vec3b>(x, y);
				frame->data[0][y * frame->linesize[0] + x] = RGB2Y(px[0], px[1], px[2]);
			}
		}

		// Cb and Cr 
		for (int y = 0; y < c->height/2; y++) {
			for (int x = 0; x < c->width/2; x++) {
				cv::Vec3b px = (image.at<cv::Vec3b>(x, y) + image.at<cv::Vec3b>(x+1, y)
				 + image.at<cv::Vec3b>(x, y+1) + image.at<cv::Vec3b>(x+1, y+1)) / 4;
				frame->data[1][y * frame->linesize[1] + x] =  RGB2U(px[0], px[1], px[2]);
				frame->data[2][y * frame->linesize[2] + x] =  RGB2V(px[0], px[1], px[2]);
			}
		}
		
		for (got_output = true; got_output;) {
			if (avcodec_encode_video2(c, &pkt, frame, &got_output) < 0) {
				fprintf(stderr, "Error encoding frame\n");
				exit(1);
			}
			if (got_output) {
				// do something with pkt.data
				av_free_packet(&pkt);
			}
		}
	}
};

int main(int argc, char** argv) {
	cv::VideoCapture camera;

	bool success = false;
		 while (!success) {
			success = camera.open("/dev/video0");
			 cout << "camera opening " << (success? "succeeded" : "failed") << endl;
			if (!success) usleep(200000); // 200 ms
		}
	
	cv::Mat image;

	while (!camera.read(image)) {
			usleep(5000);
	}
	cout << "Got first frame" << endl;
	Encoder encoder(image.rows, image.cols);

	while (true) {
		encoder.encode_frame(image);
		if (!camera.read(image)) {
			usleep(5000);
		}		  
	}
}

