#!/bin/bash
ssh pi@10.126.102.33 "cd ~/vision-code/ && rm -r src/"
scp -r `dirname $0`"/src" pi@10.126.102.33:./vision-code/src
ssh pi@10.126.102.33 'cd ~/vision-code/ && g++ -g -o 5708-vision src/*.cpp `pkg-config --libs opencv libavcodec libavutil`'