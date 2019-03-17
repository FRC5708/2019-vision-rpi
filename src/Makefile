
COMMON_FLAGS=-Wno-psabi -fopenmp -march=native -mcpu=native -mtune=native
CXXFLAGS=-g -Wall --std=c++11 -ffast-math $(COMMON_FLAGS) -I/usr/local/include/opencv4/

ODIR=obj

%.o: %.cpp %.hpp
	g++ -c -o $@ $< $(CXXFLAGS)

build: main.o RedContourGrip.o vision.o streamer.o DataComm.o
	g++ -Wno-psabi $(COMMON_FLAGS) -o ../5708-vision main.o RedContourGrip.o vision.o streamer.o DataComm.o `pkg-config --libs opencv` -pthread

install:
	cp ../5708-vision ../5708-vision-copy
	mv ../5708-vision-copy ~/bin/5708-vision

clean:
	rm ./*.o
