#!/bin/bash

ID=1

for i in 128 142 138 114 109 113 149 131
do
	ssh wiselab-nuc$ID@192.168.1.$i pcs-camera-server &
	ID=$(echo $ID + 1 | bc)
done

sleep 2
pcs-multicamera-client $1
#wait

ID=1

for i in 128 142 138 114 109 113 149 131
do
	ssh wiselab-nuc$ID@192.168.1.$i pkill -f pcs-camera-server
	ID=$(echo $ID + 1 | bc)
done