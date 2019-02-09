
CXXFLAGS=-g -Wall --std=c++11 `pkg-config --cflags opencv`

ODIR=obj

%.o: %.cpp
	g++ -c -o $@ $< $(CXXFLAGS)

build: main.o RedContourGrip.o vision.o streamer.o
	g++ -o ../5708-vision main.o RedContourGrip.o vision.o streamer.o `pkg-config --libs opencv` -pthread

install:
	cp ../5708-vision ~/bin/

clean:
	rm ./*.o
