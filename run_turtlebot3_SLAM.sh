#!/bin/bash

my_pid=$$
echo "My process ID is $my_pid"

echo "Launching roscore..."
roscore &
pid=$!
sleep 2s

echo "Launching rviz..."
rosrun rviz rviz -d `rospack find crowdbot_active_slam`/rviz/turtlebot.rviz &

sleep 2s

echo "Launching factor graph SLAM system..."
roslaunch crowdbot_active_slam graph_optimisation.launch robot_name:=turtlebot_real scan_topic:=scan &
pid="$pid $!"

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
