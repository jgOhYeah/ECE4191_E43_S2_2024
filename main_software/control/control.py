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
    max_movement = 0.2 # Maximum distance to move in m.
    distance_scale = 0.5 # What fraction of the calculated distance to the ball we should use.

    # Parameters measured
    dist_to_centre = abs(ball["center point"][0])
    distance = ball["distance"]
    confidence = ball["confidence"]/100

    scalar = distance_scale * (1 - weight_sideways * dist_to_centre) * (1 - weight_confidence * confidence)
    calc_dist = scalar * distance
    return min(max(min_movement, calc_dist), max_movement)

def handle_balls(args: List):
    """Handles receiving a ball message.

    Args:
        args (ki): The data contained within
    """
    logging.debug("Received a handle balls message")
    # Check if we actually need to action this message (looking for balls and not moving).
    if state == State.FOLLOWING_BALL and not odometry_current.moving:
        # In the correct state to care about the balls. Check if we actually have any balls.
        if len(args) > 0:
            # We have at least one ball to choose from.
            ball = pick_ball(args)
            # speed = 0.2
            send_move_command(step_distance(ball), rotate_to_ball(ball))
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
    global state
    state = State.RETURNING_HOME
    publish_mqtt(MQTTTopics.ODOMETRY_GO_HOME, {})

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
