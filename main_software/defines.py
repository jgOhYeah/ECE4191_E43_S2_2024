"""defines.py
This file should be included by all sections. It contains MQTT login detauls and topics."""

from typing import Callable, List
import paho.mqtt.client as mqtt
from dataclasses import dataclass
import logging
import sys
import json

MQTT_HOST = "localhost"


class MQTTTopics:
    """Class to store MQTT topics for easier access later.
    Add them / document them here: https://www.notion.so/footpatheducation/MQTT-Topics-ca218dfc54964739b4436ab341c8a80f
    """

    # Stuff to do with the odometry system.
    ODOMETRY_MOVE = "/odometry/move"
    ODOMETRY_GO_HOME = "/odometry/go-home"
    ODOMETRY_CURRENT = "/odometry/current"
    ODOMETRY_STATUS = "/odometry/status"

    # Stuff to do with the vision system.
    VISION_CONTACT = "/vision/contact"
    VISION_BALLS = "/vision/balls"
    VISION_BOUNDARIES = "/vision/boundaries"


@dataclass
class TopicMethodPair:
    """Data class that stores a topic and associated callback function to call when a message with this topic is received."""

    topic: str  # The topic assiciated with the method.
    method: Callable[
        [dict], None
    ]  # The method to call when the topic is received. The dictionary contains the JSON formatted message.


mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
_topic_pairs = []
logger = logging.getLogger(__name__)


def _on_connect(
    client: mqtt.Client, userdata: None, flags, reason_code: int, properties
) -> None:
    """Called when MQTT is connected, subscribes to all required topics.

    Args:
        client (mqtt.Client): The MQTT client.
        userdata (None): _description_
        flags (_type_): _description_
        reason_code (int): _description_
        properties (_type_): _description_
    """
    logging.info("MQTT connected")
    for tp in _topic_pairs:
        logging.info(f"  - Subscribing to topic '{tp.topic}")
        mqtt_client.subscribe(tp.topic)


def _on_message(client: mqtt.Client, userdata: None, msg: mqtt.MQTTMessage) -> None:
    """Handles a received message from MQTT.

    Args:
        client (mqtt.Client): The MQTT client.
        userdata (None): The user data.
        msg (mqtt.MQTTMessage): The message structure.
    """
    logging.debug(f"Received message on topic '{msg.topic}'")
    for tp in _topic_pairs:
        if tp.topic == msg.topic:
            logging.debug(f"  - Found, about to call method")
            try:
                args = json.loads(msg.payload)
            except json.decoder.JSONDecodeError:
                logging.warning(f"  - Could not decode message '{msg.payload}'")
            else:
                tp.method(args)


def setup_mqtt(topic_method_pairs: List[TopicMethodPair]):
    """Sets up MQTT messaging.

    Args:
        topic_method_pairs (List[TopicMethodPair]): A list of topics to subscribe to and methods that will be called when that topic is received.
    """
    global _topic_pairs
    _topic_pairs = topic_method_pairs

    logging.debug("Setting up MQTT")
    mqtt_client.on_connect = _on_connect
    mqtt_client.on_message = _on_message
    mqtt_client.connect(MQTT_HOST)


def publish_mqtt(topic: str, message: dict):
    """Publishes an MQTT message. Formats the message as a JSON object automatically.

    Args:
        topic (str): The topic to send the message on.
        message (dict): The message payload.
    """
    payload = json.dumps(message)
    logging.debug(f"Publishing on topic '{topic}'. Payload '{payload}'")
    mqtt_client.publish(topic, payload)


def setup_logging(filename: str = "log.txt", log_level: int = logging.DEBUG):
    """Sets up logging.

    Args:
        filename (str, optional): Filename to save logs to. Defaults to "log.txt".
        log_level (int, optional): Level to log at (any messages ranked below this won't be shown). Defaults to logging.DEBUG.
    """
    logging.basicConfig(
        format="%(asctime)s %(levelname)s: %(message)s",
        filename=filename,
        level=log_level,
    )
    logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))

class OdometryCurrent:
    """Class for storing, transmitting and receiving the current odometry readings."""
    def __init__(self):
        self.heading: float = 0
        self.position: List[float] = [0, 0]
        self.speed: float = 0
        self.angular_velocity: float = 0
        self.moving: bool = False

    def publish(self):
        """Publishes the current object to MQTT."""
        logging.debug("Publishing the current position and velocity.")
        publish_mqtt(
            MQTTTopics.ODOMETRY_CURRENT,
            {
                "moving": self.moving,
                "heading": self.heading,
                "position": self.position,
                "speed": self.speed,
                "angular-velocity": self.angular_velocity,
            },
        )

    def receive(self, args:dict):
        """Populates the object with a received object."""
        logging.debug("Receiving latest positions.")
        self.moving = args["moving"]
        self.heading = args["heading"]
        self.position = args["position"]
        self.speed = args["speed"]
        self.angular_velocity = args["angular-velocity"]