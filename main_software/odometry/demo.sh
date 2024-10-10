#!/usr/bin/bash
# Demonstration that drives the robot around for a bit and then stops using MQTT.
host="tennisbot.local"
echo "Moving backward"
mosquitto_pub -h $host -t "/odometry/move-speed" -m '{"angular-velocity": -0, "speed": -0.2}'
sleep 5
echo "Moving backward whilst rotating"
mosquitto_pub -h $host -t "/odometry/move-speed" -m '{"angular-velocity": -0.4, "speed": -0.2}'
sleep 5
echo "Rotating on the spot"
mosquitto_pub -h $host -t "/odometry/move-speed" -m '{"angular-velocity": 0.4, "speed": 0}'
sleep 5
echo "Moving forward"
mosquitto_pub -h $host -t "/odometry/move-speed" -m '{"angular-velocity": -0, "speed": 0.3}'
sleep 5
# mosquitto_pub -h $host -t "/odometry/move-speed" -m '{"angular-velocity": -0, "speed": 0.2}'
# sleep 5
# mosquitto_pub -h $host -t "/odometry/move-speed" -m '{"angular-velocity": 0.4, "speed": 0.2}'
# sleep 5
# mosquitto_pub -h tennisbot.local -t "/odometry/move-speed" -m '{"angular-velocity": -0.4, "speed": 0}'
# sleep 5
# mosquitto_pub -h tennisbot.local -t "/odometry/move-speed" -m '{"angular-velocity": 0, "speed": -0.3}'
# sleep 5
echo "Stopping"
mosquitto_pub -h $host -t "/odometry/move-speed" -m '{"angular-velocity": -0, "speed": 0}'
echo "Starting ball loader"
mosquitto_pub -h $host -t "/ball/start" -m '{"ball-count": 5}'