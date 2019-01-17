#!/bin/bash

g++ -g -Wall -o 5708-vision --std=c++11 `pkg-config --libs opencv4 --cflags opencv4 libavcodec libavutil` src/*.cpp