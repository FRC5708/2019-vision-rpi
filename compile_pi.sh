#!/bin/bash
PI_ADDR=10.126.102.45

rsync -rc `dirname $0`"/src" "pi@"$PI_ADDR":./vision-code/"
ssh pi@$PI_ADDR "cd ~/vision-code/src && make -j2"
