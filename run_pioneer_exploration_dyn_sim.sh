#!/bin/bash

my_pid=$$
echo "My process ID is $my_pid"

echo "Launching roscore..."
roscore &
pid=$!
sleep 2s

echo "Launching pioneer in gazebo and rviz..."
roslaunch crowdbot_active_slam pioneer_gazebo.launch &
pid="$pid $!"

sleep 10s

echo "Launching laser scan matcher..."
roslaunch crowdbot_active_slam pedestrian_simulator.launch N:=50 &
pid="$pid $!"

sleep 20s

echo "Launching factor graph SLAM system..."
roslaunch crowdbot_active_slam graph_optimisation.launch scan_topic:=static_scan_extractor/static_scan &
pid="$pid $!"

sleep 2s

echo "Launching static scan extractor..."
roslaunch crowdbot_active_slam static_scan_extractor.launch &
pid="$pid $!"

sleep 6s

echo "Launching move_base..."
roslaunch crowdbot_active_slam move_base.launch &
pid="$pid $!"

sleep 2s

echo "Launching frontier exploration..."
roslaunch crowdbot_active_slam frontier_exploration_service.launch &
pid="$pid $!"

sleep 2s

echo "Launching decision maker..."
roslaunch crowdbot_active_slam decision_maker.launch &
pid="$pid $!"

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
