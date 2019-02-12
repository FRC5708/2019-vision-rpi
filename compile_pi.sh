#!/bin/bash
if [ -z "$PI_ADDR" ]; then PI_ADDR=raspberrypi.local; fi

rsync -rc -e "ssh -p 5810" `dirname $0`"/src" "pi@"$PI_ADDR":./vision-code/"
ssh pi@$PI_ADDR -p 5810 "cd ~/vision-code/src && make build -j4 && make install"
