#!/usr/bin/bash

# Publish a message to say we are about to go.

# Start computer vision
echo "Starting CV"
cd ../CV; python3 detect_v2.py --modeldir="saved_models/tennis_model_v2" --threshold=0.15 &

# Start odometry
echo "Starting odometry TODO"
cd ./odometry

# Start the controller
echo "Starting the controller"
../.venv/bin/python3 ./control/control.py

# Kill everything on exit
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
