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
)

# Logging
import logging

logger = logging.getLogger(__name__)

# States
from enum import Enum

# class State(Enum):


def handle_balls(args: List):
    """Handles receiving a ball message.

    Args:
        args (ki): The data contained within
    """
    logging.debug("Received a handle balls message")
    logging.debug(args)
    # TODO


def handle_contact(args: dict):
    """Handles receiving a contact message.

    Args:
        args (dict): The data contained within the message.
    """
    logging.debug("Received a contact message")
    logging.debug(args)
    # TODO: Change state and all that.
    publish_mqtt(MQTTTopics.ODOMETRY_GO_HOME, {})

if __name__ == "__main__":
    # Main code to run.
    setup_logging("log_control.txt", logging.DEBUG)
    method_pairs = [
        TopicMethodPair(MQTTTopics.VISION_BALLS, handle_balls),
        TopicMethodPair(MQTTTopics.VISION_CONTACT, handle_contact),
    ]
    setup_mqtt(method_pairs)
    mqtt_client.loop_forever()  # Use mqtt_client.loop_start()
