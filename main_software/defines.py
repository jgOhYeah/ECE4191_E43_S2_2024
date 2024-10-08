"""defines.py
This file should be included by all sections. It contains MQTT login detauls and topics."""

from __future__ import annotations
from typing import Callable, List, Tuple
import paho.mqtt.client as mqtt
from dataclasses import dataclass
import logging
import sys
import json
from abc import ABC, abstractmethod

MQTT_HOST = "localhost"


class MQTTTopics:
    """Class to store MQTT topics for easier access later.
    Add them / document them here: https://www.notion.so/footpatheducation/MQTT-Topics-ca218dfc54964739b4436ab341c8a80f
    """

    # Stuff to do with the odometry system.
    ODOMETRY_MOVE_SPEED = "/odometry/move-speed"
    ODOMETRY_MOVE_POSITION = "/odometry/move-position"
    ODOMETRY_GO_HOME = "/odometry/go-home"
    ODOMETRY_CURRENT = "/odometry/current"
    ODOMETRY_STATUS = "/odometry/status"
    ODOMETRY_SENSORS = "/odometry/sensors"

    # Ball loading
    BALL_LOAD_START = "/ball/start"
    BALL_LOAD_FINISH = "/ball/finish"

    # Stuff to do with the vision system.
    VISION_CONTACT = "/vision/contact"
    VISION_BALLS = "/vision/balls"
    VISION_BOUNDARIES = "/vision/boundaries"

    # Status
    STATUS = "/status"


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
        logging.info(f"  - Subscribing to topic '{tp.topic}' ({tp.method})")
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
    # logging.debug(f"Publishing on topic '{topic}'. Payload '{payload}'")
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


class MQTTTopicImplementation(ABC):
    """Base class to aid in the implementation of MQTT topics."""

    def __init__(
        self,
        topic: MQTTTopics,
        callback: Callable[[MQTTTopicImplementation], None] = None,
    ) -> None:
        """Initialises the topic.

        Args:
            topic (MQTTTopics): The topic to publish on or subscribe to.
            callback (Callable[[MQTTTopicImplementation], None], optional): Callback function to call when receive() is called. Defaults to None.
        """
        self.topic = topic
        self.callback = callback

    @abstractmethod
    def to_dict(self) -> dict:
        """Formats the data as a dictionary inline with the MQTT topics database.

        Returns:
            dict: The data as a dictionary.
        """
        pass

    def publish(self) -> None:
        """Publishes the current data to MQTT."""
        publish_mqtt(
            self.topic,
            self.to_dict(),
        )

    @abstractmethod
    def receive(self, args: dict) -> None:
        """Receives a dictionary containing the arguments and updates the class variables.
        Assume that the dictionary was created by this object's to_dict() method."""
        if self.callback:
            self.callback(self)

    def topic_method_pair(self) -> TopicMethodPair:
        """Returns the topic and method pair for this implementation of a topic."""
        return TopicMethodPair(self.topic, self.receive)

    def __repr__(self) -> str:
        return json.dumps(f"MQTT topic '{self.topic}', '{self.to_dict()}'")


class OdometryCurrent(MQTTTopicImplementation):
    """Class for storing, transmitting and receiving the current odometry readings."""

    def __init__(
        self,
        heading: float = 0,
        position: Tuple[float, float] = (0, 0),
        speed: float = 0,
        angular_velocity: float = 0,
        moving: bool = False,
        turn_radius: float = 0,
        callback: Callable[[MQTTTopicImplementation], None] = None,
    ):
        """Creates the status object.

        All parameters are optional and default to 0 (or their equivalent.)

        Args:
            heading (float, optional): Heading in radians. Defaults to 0.
            position (Tuple[float, float], optional): Position in m. Defaults to (0, 0).
            speed (float, optional): Speed in m/s. Defaults to 0.
            angular_velocity (float, optional): Angular velocity in rad/s. Defaults to 0.
            moving (bool, optional): Whether the robot is moving. Defaults to False.
            turn_radius (float, optional): The current turning radius in m. Defaults to 0.
            callback (Callable[[MQTTTopicImplementation], None], optional): Callback function to call when receive() is called. Defaults to None.
        """
        super().__init__(MQTTTopics.ODOMETRY_CURRENT, callback)
        self.heading = heading
        self.position = position
        self.speed = speed
        self.angular_velocity = angular_velocity
        self.moving = moving
        self.turn_radius = turn_radius

    def to_dict(self) -> dict:
        """Represents the data as a dictionary.

        Returns:
            dict: The dictionary representation as given in the MQTT documentation.
        """
        return {
            "moving": bool(self.moving),
            "heading": self.heading,
            "position": self.position,
            "speed": self.speed,
            "angular-velocity": self.angular_velocity,
            "turn-radius": self.turn_radius,
        }

    def receive(self, args: dict):
        """Populates the object with a received object."""
        logging.debug("Receiving latest positions.")
        self.moving = args["moving"]
        self.heading = args["heading"]
        self.position = args["position"]
        self.speed = args["speed"]
        self.angular_velocity = args["angular-velocity"]
        self.turn_radius = args["turn-radius"]

        # Call the callback if needed.
        super().receive(args)

class MoveSpeed(MQTTTopicImplementation):
    """MQTT topic to command the robot to move with a given speed."""

    def __init__(
        self,
        angular_velocity: float = 0,
        speed: float = 0,
        callback: Callable[[MQTTTopicImplementation], None] = None,
    ):
        """Creates the move at speed object.

        Args:
            angular_velocity (float, optional): The current angular velocity of the robot rotating around the center of
                                                the driven axle. When viewed from above, clockwise is positive. Defaults
                                                to 0.
            speed (float, optional): The speed in the linear direction. Positive is forwards, negative is backwards.
                                     Defaults to 0.
            callback (Callable[[MQTTTopicImplementation], None], optional): Callback function to call when receive() is called. Defaults to None.
        """
        super().__init__(MQTTTopics.ODOMETRY_MOVE_SPEED, callback)
        self.angular_velocity = angular_velocity
        self.speed = speed
    
    def to_dict(self) -> dict:
        return {
            "angular-velocity": self.angular_velocity,
            "speed": self.speed
        }
    
    def receive(self, args: dict) -> None:
        self.angular_velocity = args["angular-velocity"]
        self.speed = args["speed"]
        return super().receive(args)

class MovePosition(MQTTTopicImplementation):
    """MQTT topic to command the robot to move with a given speed."""

    def __init__(
        self,
        angular_velocity: float = 0,
        speed: float = 0,
        position: Tuple[float, float] = (0, 0),
        callback: Callable[[MQTTTopicImplementation], None] = None,
        is_relative: bool = True
    ):
        """Creates the move at speed object.

        Args:
            angular_velocity (float, optional): The current angular velocity of the robot rotating around the center of
                                                the driven axle. When viewed from above, clockwise is positive. Defaults
                                                to 0.
            speed (float, optional): The speed in the linear direction. Positive is forwards, negative is backwards.
                                     Defaults to 0.
            position (Tuple[float, float], optional): Position in m. Defaults to (0, 0).
            callback (Callable[[MQTTTopicImplementation], None], optional): Callback function to call when receive() is called. Defaults to None.
            is_relative (bool, optional): Whether the position is relative or absolute. Defaults to True
        """
        super().__init__(MQTTTopics.ODOMETRY_MOVE_POSITION, callback)
        self.angular_velocity = angular_velocity
        self.speed = speed
        self.position = position
        self.is_relative = is_relative
    
    def to_dict(self) -> dict:
        return {
            "angular-velocity": self.angular_velocity,
            "speed": self.speed,
            "position": self.position,
            "is_relative": self.is_relative 
        }
    
    def receive(self, args: dict) -> None:
        self.angular_velocity = args["angular-velocity"]
        self.speed = args["speed"]
        self.position = args["position"]
        self.is_relative = args.get("is_relative", True)
        return super().receive(args)


class OdometrySensors(MQTTTopicImplementation):
    """Class for storing, transmitting, and receiving odometry sensor readings."""

    def __init__(
        self,
        latest_time_left: str = "Unknown time",
        latest_time_right: str = "Unknown time",
        callback: Callable[[MQTTTopicImplementation], None] = None,
    ):
        """Creates the odometry sensors object.

        Args:
            latest_time_left (str, optional): Latest time a white line was detected on the left sensor. Defaults to "Unknown time".
            latest_time_right (str, optional): Latest time a white line was detected on the right sensor. Defaults to "Unknown time".
            callback (Callable[[MQTTTopicImplementation], None], optional): Callback function to call when receive() is called. Defaults to None.
        """
        super().__init__(MQTTTopics.ODOMETRY_SENSORS, callback)
        self.latest_time_left = latest_time_left
        self.latest_time_right = latest_time_right

    def to_dict(self) -> dict:
        """Represents the sensor data as a dictionary.

        Returns:
            dict: The dictionary representation of the sensor data.
        """
        return {
            "latest_time_left": self.latest_time_left,
            "latest_time_right": self.latest_time_right,
        }

    def receive(self, args: dict) -> None:
        """Processes incoming sensor data.

        Args:
            args (dict): The incoming message payload.
        """
        sensor_time = args.get("time", "Unknown time")
        sensor_id = args.get("sensor", "unknown")

        logging.info(f"Sensor Event: line detected on {sensor_id} at {sensor_time}")

        # Update latest detection times based on which sensor is reporting
        if sensor_id == "left":
            self.latest_time_left = sensor_time
        elif sensor_id == "right":
            self.latest_time_right = sensor_time

        # Log the updated state
        logging.debug(f"Updated Times - Left: {self.latest_time_left}, Right: {self.latest_time_right}")

        # Call the callback if needed.
        super().receive(args)

class BallLoadMsg(MQTTTopicImplementation):
    def __init__(self, ball_count:int = 0, callback=None):
        super().__init__(MQTTTopics.BALL_LOAD_START, callback)
        self.ball_count = ball_count

    def to_dict(self) -> dict:
        return {
            "ball-count": self.ball_count,
        }
    
    def receive(self, args):
        self.ball_count = args["ball-count"]
        return super().receive(args)
    
class BallLoadFinishMsg(MQTTTopicImplementation):
    def __init__(self, callback=None):
        super().__init__(MQTTTopics.BALL_LOAD_FINISH, callback)

    def to_dict(self) -> dict:
        return {}
    
    def receive(self, args):
        return super().receive(args)