###### OLD STARTUP SCRIPT

# #!/usr/bin/bash

# # Publish a message to say we are about to go.
# mosquitto_pub -t '/status' -m '{"state": "starting"}'

# cwd=$(dirname $(realpath $0))
# venv_py="$cwd/../.venv/bin/python3"
#  cv_py="/usr/bin/python3"

# # # Start computer vision
# echo "Starting CV"
# # threshold=0.15
# threshold=0.10
# cd "$cwd/../CV"; python3 detect_v2.py --modeldir="saved_models/tennis_model_v2" --threshold=$threshold

# # Start odometry
# # echo "Starting odometry"
# # cd "$cwd/odometry" && $venv_py odometry.py &

# # Start the controller
# # echo "Starting the controller"
# # cd "$cwd/control" && $venv_py control.py

# # Kill everything on exit
# # trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT ERR


###### NEW STARTUP.SH SCRIPT

#!/usr/bin/bash

# Publish a message to indicate that we are starting.
mosquitto_pub -t '/status' -m '{"state": "starting"}'

# Get the current working directory.
cwd=$(dirname $(realpath $0))

# Paths to the Python interpreters in your virtual environments.
venv_py="$cwd/../.venv/bin/python3"
cv_env_py="$cwd/../CV/cv_env/bin/python3"

# Set up a trap to kill all background processes when the script exits.
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT ERR

# Start the controller first using venv.
echo "Starting the controller"
cd "$cwd/control" && $venv_py control.py &

# Start odometry next using venv.
echo "Starting odometry"
cd "$cwd/odometry" && $venv_py odometry.py &

# Start computer vision last using cv_env, with lower priority and limited CPU usage.
echo "Starting CV"
cd "$cwd/../CV"

# Adjust the CPU core numbers (e.g., 0,1,2) based on your system's available cores.

# Assign the CV process to use specific CPU cores (e.g., cores 2 and 3) and lower its priority.
nice -n 10 taskset -c 2,3 $cv_env_py YOLO_detect_singlethread.py

# Wait for all background processes to finish.
wait
