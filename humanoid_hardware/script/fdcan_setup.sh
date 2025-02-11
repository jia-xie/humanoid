#!/bin/bash

# Set down the interface and reload CAN modules
sudo ip link set down can0
sudo modprobe -r can_raw
sudo modprobe -r mttcan
sudo modprobe -r can

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Configure and bring up CAN interface with CAN FD settings
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on #loopback on #berr-reporting on
sudo ip link set up can0

ip -details link show can0

cansend can0 000#FFFFFFFFFFFFFFFD
cansend can0 001#FFFFFFFFFFFFFFFD
cansend can0 002#FFFFFFFFFFFFFFFD
cansend can0 003#FFFFFFFFFFFFFFFD
cansend can0 004#FFFFFFFFFFFFFFFD
cansend can0 005#FFFFFFFFFFFFFFFD
cansend can0 006#FFFFFFFFFFFFFFFD
cansend can0 007#FFFFFFFFFFFFFFFD
cansend can0 008#FFFFFFFFFFFFFFFD

cansend can0 7FF#0100552309000000
sleep 0.1
cansend can0 7FF#0200552309000000
sleep 0.1
cansend can0 7FF#0300552309000000
sleep 0.1
cansend can0 7FF#0400552309000000
sleep 0.1
cansend can0 7FF#0500552309000000
sleep 0.1
cansend can0 7FF#0600552309000000
sleep 0.1
cansend can0 7FF#0700552309000000
sleep 0.1
cansend can0 7FF#0800552309000000

sleep 0.2

cansend can0 000#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 001#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 002#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 003#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 004#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 005#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 006#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 007#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 008#FFFFFFFFFFFFFFFC