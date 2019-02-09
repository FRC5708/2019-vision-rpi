#!/bin/sh

if [ -z "$PI_ADDR" ]; then PI_ADDR=raspberrypi.local; fi
if [ -z "$GST_COMMAND" ]; then GST_COMMAND=gst-launch-1.0; fi

echo "" | nc $PI_ADDR 8080 && \
$GST_COMMAND udpsrc port=1234 ! gdpdepay ! rtph264depay ! avdec_h264 ! autovideosink sync=false
