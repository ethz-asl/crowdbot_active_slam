#!/bin/bash

my_pid=$$
echo "My process ID is $my_pid"

echo "Launching roscore..."
roscore &
pid=$!
sleep 2s

roslaunch pepper_description pepper_upload.launch &
sleep 2s

echo "Launching combined laser scans node..."
roslaunch crowdbot_active_slam combined_laser_scans.launch &
pid="$pid $!"

sleep 2s

echo "Launching factor graph SLAM system..."
roslaunch crowdbot_active_slam graph_optimisation_pepper.launch robot_name:=pepper_real scan_topic:=static_scan_extractor/static_scan &
pid="$pid $!"

sleep 2s

echo "Launching static scan extractor..."
roslaunch crowdbot_active_slam static_scan_extractor.launch scan_topic:=/combined_scan_sync &
pid="$pid $!"

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
