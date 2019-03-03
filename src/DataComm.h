#pragma once
#include <vector>
#include <chrono>
#include "vision.hpp"
class DataComm{
    int fd=-1;
    const char* client_name;
    std::chrono::steady_clock clock;
public:
    void setupSocket();
    DataComm(const char* client_name);
    void sendData(std::vector<VisionData> data, std::chrono::time_point<std::chrono::steady_clock> timeFrom);

};