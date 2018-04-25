#!/bin/bash
echo "Testin"
source /opt/ros/kinetic/setup.bash
source ~/kuka_catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

rqt --standalone rqt_kuka &
sleep 1
rviz
sleep 3000
