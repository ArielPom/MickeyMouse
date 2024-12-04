#!/bin/bash

stream_url="localhost:8554/stream1"

quality_options="--width 1920 --height 1080 --bitrate 12000000"
# quality_options=""

ffmpeg_options="-analyzeduration 0 -fflags nobuffer -flags low_delay -i - -c:v copy -preset ultrafast -tune zerolatency"
libcamera-vid -t 0 ${quality_options} --inline -n -o - | ffmpeg ${ffmpeg_options} -f rtsp -rtsp_transport udp rtsp://${stream_url}