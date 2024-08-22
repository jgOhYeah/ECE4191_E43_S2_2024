#!/usr/bin/env python3
"""Does computer vision stuff and sends relevent MQTT messages"""
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


if __name__ == "__main__":
    # Main code to run.
    setup_logging("log_vision.txt", logging.DEBUG)
    method_pairs = [
        # TODO
    ]
    setup_mqtt(method_pairs)
    mqtt_client.loop_forever()  # Use mqtt_client.loop_start() to not block.
