#!/usr/bin/env python3
"""Accepts inputs from vision system and sensors, calculates next course of action, sends commands over MQTT to rest of robot."""
# Typing and pretty stuff
from typing import List

# Import the helper functions in defines.py
import sys

sys.path.insert(1, sys.path[0] + "/../")
from defines import (
    setup_logging,
    setup_mqtt,
    MQTTTopics,
    TopicMethodPair,
    mqtt_client,
    publish_mqtt,
    OdometryCurrent
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

def send_move_command(speed: float, heading: float):
    """Publishes a command to move.

    Args:
        speed (float): The speed to move with.
        heading (float): The global heading to point at.
    """
    publish_mqtt(MQTTTopics.ODOMETRY_MOVE, {"speed": speed, "heading": heading})


def pick_ball(balls: List[dict]):
    """Picks a ball to follow / track from a list of balls.

    Args:
        balls (List[dict]): The balls to select from.
    """
    # Closest ball to centre.
    closest_ball = None
    closest_dist = math.inf
    for ball in balls:
        dist_to_centre = abs(ball["coords"][0])
        if closest_dist > dist_to_centre:
            closest_dist = dist_to_centre
            closest_ball = ball
    
    return closest_ball

def rotate_to_ball(ball:dict):
    """Calculates the heading towards a ball.

    Args:
        ball (dict): The ball to track.
    """
    # TODO: Work out correct camera field of view / angles?
    current = odometry_current.heading
    return current + (ball["coords"][0] * math.pi/4)

def rotate_heading() -> float:
    """Calculates the new heading to rotate to find a ball."""
    # TODO: Camera field of view?
    return odometry_current.heading + math.pi/4

def handle_balls(args: List):
    """Handles receiving a ball message.

    Args:
        args (ki): The data contained within
    """
    logging.debug("Received a handle balls message")
    logging.debug(args)
    logging.debug(type(args))
    # Check if we actually need to action this message
    if state == State.FOLLOWING_BALL:
        # In the correct state to care about the balls. Check if we actually have any balls.
        if len(args) > 0:
            # We have at least one ball to choose from.
            ball = pick_ball(args)
            speed = 0.2
            send_move_command(speed, rotate_to_ball(ball))
        else:
            # We can't find any balls.
            send_move_command(0, rotate_heading())
    else:
        logging.debug("Don't need to handle balls.")


def handle_contact(args: dict):
    """Handles receiving a contact message.

    Args:
        args (dict): The data contained within the message.
    """
    logging.debug("Received a contact message")
    logging.debug(args)
    global state
    state = State.RETURNING_HOME
    publish_mqtt(MQTTTopics.ODOMETRY_GO_HOME, {})

def update_current(args:dict):
    """Updates the current position and heading.

    Args:
        args (dict): The current position and heading.
    """
    global position
    global heading
    position = args["position"]

if __name__ == "__main__":
    # Main code to run.
    setup_logging("log_control.txt", logging.DEBUG)
    method_pairs = [
        TopicMethodPair(MQTTTopics.VISION_BALLS, handle_balls),
        TopicMethodPair(MQTTTopics.VISION_CONTACT, handle_contact),
        TopicMethodPair(MQTTTopics.ODOMETRY_CURRENT, odometry_current.receive)
    ]
    setup_mqtt(method_pairs)
    mqtt_client.loop_forever()  # Use mqtt_client.loop_start()
