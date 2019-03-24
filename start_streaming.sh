#!/bin/sh

if [ -z "$PI_ADDR" ]; then PI_ADDR=raspberrypi.local; fi
if [ -z "$BITRATE" ]; then BITRATE=1000000; fi

if [ -z "$GST_COMMAND" ]; then 
    if [ -f /proc/version ] && grep -q "Microsoft" /proc/version; then
        GST_COMMAND="/mnt/c/gstreamer/1.0/x86_64/bin/gst-launch-1.0.exe"
    else
        GST_COMMAND="gst-launch-1.0"
    fi
fi
#timeout 1 nc -ul4 1234 &
#timeout 1 nc -ul6 1234 &

if echo $BITRATE | nc  $PI_ADDR 5807; then
    $GST_COMMAND udpsrc port=5809 ! gdpdepay ! rtph264depay ! avdec_h264 ! autovideosink sync=false
else
    echo "Could not connect to rPi"
    exit 1
fi
