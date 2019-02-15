
COMMON_FLAGS=-Wno-psabi -fopenmp -march=native -mcpu=native -mtune=native
CXXFLAGS=-g -Wall --std=c++11 -ffast-math $(COMMON_FLAGS) `pkg-config --cflags opencv`

ODIR=obj

%.o: %.cpp
	g++ -c -o $@ $< $(CXXFLAGS)

build: main.o RedContourGrip.o vision.o streamer.o
	g++ -Wno-psabi $(COMMON_FLAGS) -o ../5708-vision main.o RedContourGrip.o vision.o streamer.o `pkg-config --libs opencv` -pthread

install:
	cp -f ../5708-vision ~/bin/

clean:
	rm ./*.o
