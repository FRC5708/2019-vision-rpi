cmake_minimum_required(VERSION 3.10)

# set the project name
project(5708-vision)

file(GLOB src_files ${PROJECT_SOURCE_DIR}/*.cpp)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# add the executable
add_executable(5708-vision ${src_files})
target_link_libraries(5708-vision PRIVATE ${OpenCV_LIBRARIES} Threads::Threads)
