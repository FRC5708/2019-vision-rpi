#include <opencv2/opencv.hpp>


/*
For streaming:
Use mkfifo
then use VLC to broadcast a RTP stream to 239.0.0.1 (https://wiki.videolan.org/Documentation:Streaming_HowTo/Advanced_Streaming_Using_the_Command_Line/#rtp)
then use VLC on driver console to connect to that stream
*/
class Encoder {
public:
    AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    AVCodecContext *c;
    AVFrame *frame = avcodec_alloc_frame();
    AVPacket pkt;

    Encoder(int width, int height) {
        c = avcodec_alloc_context3(codec);
        c->bit_rate = 3000000; // 3 megabits 

        c->width = width;
        c->height = height;

        avcodec_open2(c, codec, nullptr);

		frame->format = c->pix_fmt;
     	frame->width  = c->width;
		frame->height = c->height;


    }
    void encode_frame(cv::Mat image) {
		bool got_output;

        av_init_packet(&pkt);
        pkt.data = NULL;
        pkt.size = 0;

		frame->data = image->data; // Might need some sort of conversion

		
		for (got_output = 1; got_output; i++) {
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
}

int main(int argc, char** argv) {
    cv:VideoCapture camera;
	
    cv:Mat image;

	camera.read(image);

	Encoder encoder(image.rows, image.cols);

    while (true) {
		encoder.encode_frame(image);
        camera.read(image);  
    }
}

