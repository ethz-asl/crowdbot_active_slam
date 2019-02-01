#!/bin/bash

my_pid=$$
echo "My process ID is $my_pid"

echo "Launching roscore..."
roscore &
pid=$!
sleep 2s

echo "Launching combined laser scans node..."
roslaunch crowdbot_active_slam combined_laser_scans.launch &
pid="$pid $!"

sleep 10s

echo "Launching laser scan matcher..."
roslaunch crowdbot_active_slam scan_matcher_pepper.launch scan_topic:=combined_scan_sync &
pid="$pid $!"

sleep 2s

echo "Launching factor graph SLAM system..."
roslaunch crowdbot_active_slam graph_optimisation.launch using_gazebo:=false scan_topic:=combined_scan_sync &
pid="$pid $!"

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
