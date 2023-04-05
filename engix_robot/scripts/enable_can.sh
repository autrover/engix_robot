#!/bin/bash
sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo busybox devmem 0x0c303010 32 0x0000C400
sudo busybox devmem 0x0c303018 32 0x0000C458

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 up type can bitrate 250000 dbitrate 1000000 berr-reporting on fd on
#sudo ip link set can1 up type can bitrate 250000 dbitrate 1000000 berr-reporting on fd on
#sudo ip link set can0 type can bitrate 250000 berr-reporting on fd on
#sudo ip link set can1 type can bitrate 250000 dbitrate 2000000 berr-reporting on fd on
#sudo ip link set can0 type can bitrate 250000
#sudo ip link set can1 type can bitrate 250000
sudo ifconfig can0 txqueuelen 15000
#sudo ifconfig can1 txqueuelen 15000
#sudo ip link set can0 up type can bitrate 250000 dbitrate 2000000 fd on fd-non-iso on
#sudo ip link set can1 up type can bitrate 250000 dbitrate 2000000 fd on fd-non-iso on

#sudo ip link set can0 type can bitrate 250000

#sudo ip link set can0 up txqueuelen 250000 type can bitrate 250000
#sudo ip link set can1 up txqueuelen 250000 type can bitrate 250000

sudo ip link set up can0
#sudo ip link set up can1

exit 0
