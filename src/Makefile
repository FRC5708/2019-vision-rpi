
CXXFLAGS=-g -Wall --std=c++11 `pkg-config --cflags opencv`

ODIR=obj

%.o: %.cpp
	g++ -c -o $@ $< $(CXXFLAGS)

all: main.o grip.o vision.o streamer.o
	g++ -o ../5708-vision main.o grip.o vision.o streamer.o `pkg-config --libs opencv` -pthread

clean:
	rm ./*.o
