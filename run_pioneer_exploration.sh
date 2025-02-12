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

echo "Launching factor graph SLAM system..."
roslaunch crowdbot_active_slam graph_optimisation.launch &
pid="$pid $!"

sleep 2s

echo "Launching move_base..."
roslaunch crowdbot_active_slam move_base.launch dynamic:=static &
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
