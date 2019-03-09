#!/bin/sh

if [ -z "$PI_ADDR" ]; then PI_ADDR=raspberrypi.local; fi
if [ -z "$BITRATE" ]; then BITRATE=1000000; fi

if [ -z "$GST_COMMAND" ]; then 
    if grep -q "Microsoft" /proc/version; then
        GST_COMMAND="/mnt/c/gstreamer/1.0/x86_64/bin/gst-launch-1.0.exe"
    else
        GST_COMMAND="gst-launch-1.0"
    fi
fi
#timeout 1 nc -ul4 1234 &
#timeout 1 nc -ul6 1234 &

echo $BITRATE | nc -4 $PI_ADDR 5807 && \
$GST_COMMAND udpsrc port=5809 ! gdpdepay ! rtph264depay ! avdec_h264 ! autovideosink sync=false
