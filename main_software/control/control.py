#!/usr/bin/env python3
"""Accepts inputs from vision system and sensors, calculates next course of action, sends commands over MQTT to rest of robot."""
# Typing and pretty stuff
from typing import List

# Import the helper functions in defines.py
import sys
from datetime import datetime
import time

sys.path.insert(1, sys.path[0] + "/../")
from defines import (
    setup_logging,
    setup_mqtt,
    MQTTTopics,
    TopicMethodPair,
    mqtt_client,
    publish_mqtt,
    OdometryCurrent,
    OdometrySensors
)

# Logging
import logging

logger = logging.getLogger(__name__)

# States
from enum import Enum, auto

class State(Enum):
    """Enumerator for keeping track of the current state."""

    FOLLOWING_BALL = auto()
    RETURNING_HOME = auto()

state = State.FOLLOWING_BALL

# Import other libraries
import math

odometry_current = OdometryCurrent()
odometry_sensors = OdometrySensors()

class ContactChecker:
    """Class for detecting if a contact has occurred based on distance."""
    def __init__(self, callback, length:int=5, threshold:float=0.2):
        self.history = []
        self.callback = callback
        self.length = length
        self.threshold = threshold
    
    def update(self, distance:float) -> bool:
        """Adds a new distance. Calls the callback if contact has occurred.

        Args:
            distance (float): The distance to add.
        """
        self.history.append(distance)
        self.history = self.history[-self.length:] # only keep the last 5.
        if self.is_contact():
            logging.info("Contact has occurred")
            self.callback({})
            return True
        else:
            return False
    
    def is_contact(self) -> bool:
        """Checks if the past few readings are close enough for a contact.

        Returns:
            bool: True if contact, False if not.
        """
        return (len(self.history) == self.length) and ((sum(self.history) / self.length) < self.threshold)

def send_move_command(distance: float, heading: float):
    """Publishes a command to move.

    Args:
        distance (float): The distance to move.
        heading (float): The global heading to point at.
    """
    global state
    odometry_current.moving = True # Pre-emptively mark as moving to avoid race conditions.
    publish_mqtt(MQTTTopics.ODOMETRY_MOVE, {"distance": distance, "heading": heading})

def score_ball(ball:dict):
    """Calculates a score to assign to a ball when picking one"""
    # Weights
    weight_centre = 0.6
    weight_dist = 0.3
    weight_confidence = 0.1

    # Parameters measured
    dist_to_centre = abs(ball["center point"][0])
    distance = ball["distance"]
    confidence = ball["confidence"]/100
    return weight_centre * dist_to_centre + weight_dist * distance + weight_confidence * confidence

def pick_ball(balls: List[dict]):
    """Picks a ball to follow / track from a list of balls.

    Args:
        balls (List[dict]): The balls to select from.
    """
    # Closest ball to centre.
    closest_ball = None
    best_score = -1
    for ball in balls:
        score = score_ball(ball)
        if score > best_score:
            best_score = score
            closest_ball = ball
    
    return closest_ball

def rotate_to_ball(ball:dict):
    """Calculates the heading towards a ball.

    Args:
        ball (dict): The ball to track.
    """
    # # TODO: Work out correct camera field of view / angles?
    # current = odometry_current.heading
    # return current + (ball["coords"][0] * math.pi/4)
    angle_scale = 0.5 # Edge of the frame is approximately this many radians.
    distance_scale = 1 # Rotate smaller ammounts the closer we get.
    return ball["center point"][0] * angle_scale * min(ball["distance"] * distance_scale, 1)
    
def rotate_heading() -> float:
    """Calculates the new heading to rotate to find a ball."""
    # TODO: Camera field of view?
    return 5*math.pi/180

def step_distance(ball:dict) -> float:
    """Calculates the distance to move each time. Use this in conjunction with rotate_to_ball.

    Args:
        ball (dict): The ball to go towards.

    Returns:
        float: The distance to move.
    """
    weight_sideways = 0.4 # Move less distance if the ball is on the edge of frame (1 = act more on this).
    weight_confidence = 0.3 # Move less distance if the confidence is low (1 = act more on this).
    min_movement = 0.01 # Minimum distance to move in m.
    max_movement = 0.4 # Maximum distance to move in m.
    distance_scale = 0.5 # What fraction of the calculated distance to the ball we should use.

    # Parameters measured
    dist_to_centre = abs(ball["center point"][0])
    distance = ball["distance"]
    confidence = ball["confidence"]/100

    scalar = distance_scale * (1 - weight_sideways * dist_to_centre) * (1 - weight_confidence * confidence)
    calc_dist = scalar * distance
    return min(max(min_movement, calc_dist), max_movement)


def handle_contact(args: dict):
    """Handles receiving a contact message.

    Args:
        args (dict): The data contained within the message.
    """
    logging.debug("Received a contact message")
    global state
    state = State.RETURNING_HOME
    publish_mqtt(MQTTTopics.ODOMETRY_GO_HOME, {})
    publish_mqtt(MQTTTopics.STATUS, {"state": "contacted, going home"})

contact = ContactChecker(handle_contact, 3, 0.2)

def handle_balls(args: List):
    """Handles receiving a ball message.

    Args:
        args (ki): The data contained within
    """
    logging.debug("Received a handle balls message")
    # Check if we actually need to action this message (looking for balls and not moving).
    if state == State.FOLLOWING_BALL:
        # Pick a ball from the frame
        if len(args) > 0:
            # We have at least one ball to choose from.
            ball = pick_ball(args)
            logging.debug(f"Picked a ball {ball}")
            publish_mqtt(MQTTTopics.STATUS, {"state": "ball found"})
            if not contact.update(ball["distance"]) and not odometry_current.moving:
                dist = step_distance(ball)
                head = rotate_to_ball(ball)
                logging.debug(f"Moving towards ball {dist=}, {head=}")
                send_move_command(dist, head)
        
        elif not odometry_current.moving:
            # Not moving, but can't find ball.
            logging.debug("Can't find a ball and not moving")
            publish_mqtt(MQTTTopics.STATUS, {"state": "no balls, moving"})
            send_move_command(0, rotate_heading())
        else:
            publish_mqtt(MQTTTopics.STATUS, {"state": "no balls, currently moving"})
            logging.debug("No balls, currently moving")
    else:
        logging.debug("Going home, don't need to handle balls.")

# Function to handle the line detection based on the latest sensor times
def handle_line_detection(sensor_data: OdometrySensors):
    """Handles line detection based on the latest sensor detection times.
    
    Args:
        sensor_data (OdometrySensors): The data containing latest detection times for both sensors.
    """
    latest_time_left = sensor_data.latest_time_left
    latest_time_right = sensor_data.latest_time_right

    # Check if both sensors have detected the line
    if latest_time_left != "Unknown time" and latest_time_right != "Unknown time":
        # Convert strings to datetime objects
        left_sensor_time = datetime.strptime(latest_time_left, '%Y-%m-%d %H:%M:%S')
        right_sensor_time = datetime.strptime(latest_time_right, '%Y-%m-%d %H:%M:%S')

        # Calculate the time difference and angle
        calculate_and_move_based_on_angle(left_sensor_time, right_sensor_time)
    else:
        logging.info("Waiting for both sensors to detect the line.")

def calculate_and_move_based_on_angle(left_sensor_time, right_sensor_time):
    """Calculates the angle of the line and sends a movement command based on detection times."""
    # Time difference in seconds
    time_diff = (right_sensor_time - left_sensor_time).total_seconds()

    # Calculate the angle based on time difference and known sensor distance
    angle = calculate_angle_from_time_difference(time_diff, sensor_distance=10, speed=odometry_current.speed)

    # Determine whether to adjust for a left or right turn
    if time_diff > 0:
        logging.info(f"Line is angled to the right by {angle} degrees")
    else:
        logging.info(f"Line is angled to the left by {-angle} degrees")

    # Calculate the opposite angle to turn the robot inwards
    opposite_angle = calculate_opposite_angle(angle)
    logging.info(f"Turning to face inward opposite to the line: {opposite_angle} degrees")

    # Send a movement command to turn to the opposite angle
    send_move_command(distance=0, heading=opposite_angle)

    # After turning, move forward 10 cm to clear the line
    send_move_command(distance=0.1, heading=opposite_angle)
    time.sleep(5) ## this is to make sure it doesnt detect the line while it is returning inside the box - hopefully?

    # Reset detection times to avoid recalculating based on old data
    reset_detection_times()

def reset_detection_times():
    """Resets the latest detection times for both sensors."""
    logging.info("Resetting detection times.")
    # This function resets the detection times in the sensor system (if needed)
    # Depending on how you track these in the `OdometrySensors` class, you could reset here or handle this separately.

def calculate_angle_from_time_difference(time_diff, sensor_distance, speed):
    """Calculates the angle from time difference, sensor distance, and robot speed."""
    import math
    return math.degrees(math.atan((time_diff * speed) / sensor_distance))

def calculate_opposite_angle(line_angle: float) -> float:
    """Calculates the opposite angle of the line.
    
    Args:
        line_angle (float): The angle of the line in degrees.

    Returns:
        float: The opposite angle in degrees.
    """
    # Normalize the angle to be within 0-360 degrees
    opposite_angle = (line_angle + 180) % 360
    return opposite_angle

def mqtt_handle_sensors(sensor_data: OdometrySensors):
    """Handles incoming sensor data from the OdometrySensors class."""
    handle_line_detection(sensor_data)

if __name__ == "__main__":
    # Main code to run.
    setup_logging("log_control.txt", logging.DEBUG)
    method_pairs = [
        TopicMethodPair(MQTTTopics.VISION_BALLS, handle_balls),
        # TopicMethodPair(MQTTTopics.VISION_CONTACT, handle_contact), # NOTE: Just doing contact in this control script for now.
        TopicMethodPair(MQTTTopics.ODOMETRY_CURRENT, odometry_current.receive),
        TopicMethodPair(MQTTTopics.ODOMETRY_SENSORS, odometry_sensors.receive)
    ]
    setup_mqtt(method_pairs)
    publish_mqtt(MQTTTopics.STATUS, {"state": "control running"})
    mqtt_client.loop_forever()  # Use mqtt_client.loop_start()
