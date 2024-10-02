#!/usr/bin/bash

# Publish a message to say we are about to go.
mosquitto_pub -t '/status' -m '{"state": "starting"}'

cwd=$(dirname $(realpath $0))
venv_py="$cwd/../.venv/bin/python3"
# cv_py="/usr/bin/python3"

# # Start computer vision
echo "Starting CV"
cd "$cwd/../CV"; python3 detect_v2.py --modeldir="saved_models/tennis_model_v2" --threshold=0.15

# Start odometry
# echo "Starting odometry"
# cd "$cwd/odometry" && $venv_py odometry.py &

# Start the controller
# echo "Starting the controller"
# cd "$cwd/control" && $venv_py control.py

# Kill everything on exit
# trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT ERR
