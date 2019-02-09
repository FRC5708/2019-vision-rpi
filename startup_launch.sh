#!/bin/bash

# run on startup
# relaunches vision code if it crashes

EXEC_PATH=/home/pi/bin/5708-vision

while [ true ]; do 
    
    killall ffmpeg
    killall gst-launch-1.0

    if [ ! -f $EXEC_PATH ]; then exit; fi
    $EXEC_PATH

done