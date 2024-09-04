#!/usr/bin/env python3
"""Uses the rotatry encoders and any other sensor data to calculate current position, velocity and heading.
Waits for MQTT message to move and actions it."""
# Typing and pretty stuff
from typing import List, Tuple

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
    OdometryCurrent,
)

# Logging
import logging

logger = logging.getLogger(__name__)

# Import necessary modules for odometry
from typing import List
import sys
import json
import logging
import time
import math
from gpiozero import Motor, RotaryEncoder
import threading
from dataclasses import dataclass
from enum import Enum
from abc import ABC, abstractmethod

# Global variables for odometry
x = 0.0
y = 0.0
theta = 0.0
left_count = 0
right_count = 0
wheel_radius = 0.027  # in meters
wheel_base = 3.05 * 0.222  # distance between wheels in meters
ticks_per_revolution = 19 * 47  #

class Side(ABC):
    """Base class for controlling the motor and encoder on a side."""

    @abstractmethod
    def __init__(
        self,
        motor_a: int,
        motor_b: int,
        motor_en: int,
        enc_a: int,
        enc_b: int,
        name: str = "",
    ):
        """Initialises the side (motor and encoder pair).

        Args:
            motor_a (int): Pin
            motor_b (int): Pin
            motor_en (int): Pin
            enc_a (int): Pin
            enc_b (int): Pin
        """
        self.motor = Motor(motor_a, motor_b, enable=motor_en, pwm=True)
        self.encoder = RotaryEncoder(enc_a, enc_b, max_steps=0)
        self.name = name
        self.target_steps = 0
        self.movement_complete = True

    @abstractmethod
    def _drive_to_steps(self, steps: int, speed: float = 1):
        """Drives the motor to a given number of steps.

        Args:
            steps (int): The number of steps (absolute, not relative).
            speed (float, optional): How fast to move. Defaults to 1.
        """
        pass

    def _drive_to_angle(self, angle: float, speed: float = 1):
        """Drives the motor to a given absolute angle.

        Args:
            angle (float): The number of radians (absolute, not relative).
            speed (float, optional): How fast to move. Defaults to 1.
        """

        self._drive_to_steps(self._angle_to_steps(angle), speed)

    def _drive_to_angle_relative(self, angle: float, speed: float = 1):
        """Drives the motor to a given angle relative to the current position.

        Args:
            angle (float): The number of radians (relative, not absolute).
            speed (float, optional): How fast to move. Defaults to 1.
        """
        abs_steps = (
            self._angle_to_steps(angle) + self.target_steps
        )  # self.target_steps assumes that the last target was reached, self.encoder.steps accrues errors.
        self._drive_to_steps(abs_steps, speed)

    def drive_to_dist(self, dist: float, speed: float = 1):
        """Drives the motor to a given absolute distance.

        Args:
            distance (float): The number of m (absolute, not relative).
            speed (float, optional): How fast to move. Defaults to 1.
        """
        self._drive_to_angle(self._dist_to_angle(dist), speed)

    def drive_to_dist_relative(self, dist: float, speed: float = 1):
        """Drives the motor to a given distance relative to the current position.

        Args:
            distance (float): The number of m (relative, not absolute).
            speed (float, optional): How fast to move. Defaults to 1.
        """
        self._drive_to_angle_relative(self._dist_to_angle(dist), speed)

    def _dist_to_angle(self, dist: float) -> float:
        return dist / wheel_radius

    def _angle_to_steps(self, angle: float) -> int:
        return int(angle / (2 * math.pi) * ticks_per_revolution)

    def _steps_to_angle(self, steps: int) -> float:
        return steps / ticks_per_revolution * 2 * math.pi

    def target_reached(self) -> bool:
        """Checks if the motors have reached their target.

        Returns:
            bool: Are we there yet?
        """
        return self.movement_complete

    @abstractmethod
    def stop(self):
        """Stops the motor"""
        pass


class SimpleSide(Side):
    """A simple motor controller that runs the motor at constant speed and stops when it reaches the correct position.
    There is no speed feedback or PID control, so there is a bit of overshoot."""

    def __init__(
        self,
        motor_a: int,
        motor_b: int,
        motor_en: int,
        enc_a: int,
        enc_b: int,
        name: str = "",
    ):
        """Initialises the side (motor and encoder pair).

        Args:
            motor_a (int): Pin
            motor_b (int): Pin
            motor_en (int): Pin
            enc_a (int): Pin
            enc_b (int): Pin
        """
        super().__init__(motor_a, motor_b, motor_en, enc_a, enc_b, name)
        self.direction = 1
        self.encoder.when_rotated = self._are_we_there_yet

        # TODO
        # self.thread = MotorControlThread(self)
        # self.thread.start()

    def _drive_to_steps(self, steps: int, speed: float = 1):
        """Drives the motor to a given number of steps.

        Args:
            steps (int): The number of steps (absolute, not relative).
            speed (float, optional): How fast to move. Defaults to 1.
        """
        logging.debug(
            f"drive_to_steps: {self.name} driving to {steps} ({self._steps_to_angle(steps)} radians)"
        )
        self.movement_complete = False
        self.target_steps = steps
        if steps > self.encoder.steps:
            # Need to go forwards
            self.motor.forward(speed)
            self.direction = 1
        else:
            # Need to go backwards
            self.motor.backward(speed)
            self.direction = -1

    def _are_we_there_yet(self):
        distance_to_go = self.direction * (self.target_steps - self.encoder.steps)
        if distance_to_go < 0:
            # We got there. Stop
            self.stop()
            # logging.debug(f"Motor stopped {distance_to_go=}")
            # TODO: Callback or something for when the motor is stopped.
            self.movement_complete = True  # Indicate that the movement is complete

    def stop(self):
        """Stops the motor"""
        # self.motor.forward(0)
        self.motor.stop()


class PISide(Side):
    """Motor controller that uses PI controllers to more intelligently control speed and position."""

    # TODO
    pass


class MovementType(Enum):
    """Enumerator for each type of movement"""

    TURN = 0
    MOVE = 1


@dataclass
class MovementRecord:
    """A record of a movement"""

    type: MovementType
    value: float


class Vehicle:
    def __init__(self):
        self.left = SimpleSide(6, 5, 13, 19, 26, "left")
        self.right = SimpleSide(16, 7, 12, 20, 21, "right")

        # TODO
        # self.odometry_thread = threading.Thread(target=self.odometry_loop)
        # self.odometry_thread.daemon = True  # Daemon thread exits when the main program exits
        # self.odometry_thread.start()

        # Initialize PI controllers for heading and distance control
        # TODO
        # self.heading_controller = PIController(kp=1.0, ki=0.1, sample_time=0.1)
        # self.distance_controller = PIController(kp=1.0, ki=0.1, sample_time=0.1)

        # Initialize movement history stack - needed if returning to origin by reversing
        self.movement_history = []
        self.status = OdometryCurrent()

    def record_movement(self, mtype: MovementType, value: float):
        """Records a movement action and its value - needed if returning to origin by reversing.

        Args:
            mtype (MovementType): the type of movement.
            value (float): how much it moved.
        """
        self.movement_history.append(MovementRecord(mtype, value))

    def odometry_loop(self):
        while True:
            self.update_odometry()
            time.sleep(1)  # Adjust the sleep duration as necessary

    def update_odometry(self):
        # global x, y, theta, left_count, right_count
        logging.debug(
            f"update_odometry: self.left.encoder.steps {self.left.encoder.steps}"
        )
        logging.debug(
            f"update_odometry: self.right.encoder.steps {self.right.encoder.steps}"
        )
        # left_distance = (2 * math.pi * wheel_radius * self.left.encoder.steps) / ticks_per_revolution
        # right_distance = (2 * math.pi * wheel_radius * self.right.encoder.steps) / ticks_per_revolution
        # distance = (left_distance + right_distance) / 2.0
        # delta_theta = (right_distance - left_distance) / wheel_base
        # x += distance * math.cos(theta + delta_theta / 2.0)
        # y += distance * math.sin(theta + delta_theta / 2.0)
        # theta += delta_theta
        # logger.debug(f"update_odometry: Position: x={x:.2f}, y={y:.2f}, theta={theta:.2f} radians")
        global x, y, theta, left_count, right_count

        # Calculate the distance traveled by each wheel
        delta_left = self.left.encoder.steps - left_count
        delta_right = self.right.encoder.steps - right_count

        # Update the counts
        left_count = self.left.encoder.steps
        right_count = self.right.encoder.steps

        # Convert steps to distance
        left_distance = delta_left * (2 * math.pi * wheel_radius) / ticks_per_revolution
        right_distance = (
            delta_right * (2 * math.pi * wheel_radius) / ticks_per_revolution
        )

        # Calculate the change in orientation
        delta_theta = (right_distance - left_distance) / wheel_base

        # Calculate the change in position
        average_distance = (left_distance + right_distance) / 2
        x += average_distance * math.cos(theta + delta_theta / 2)
        y += average_distance * math.sin(theta + delta_theta / 2)
        theta += delta_theta

        # Normalize theta to be within -pi to pi
        theta = (theta + math.pi) % (2 * math.pi) - math.pi

        # Logging for debugging
        logging.debug(
            f"update_odometry: Position: x={x:.2f}, y={y:.2f}, theta={theta:.2f} radians"
        )

    def move_to_heading(self, heading: float, distance: int, speed: float = 0.3):
        """Moves the platform to a specific relative heading and position.

        Args:
            heading (float): The new heading in radians, positive is clockwise when viewed from above.
            distance (int): Distance to move in m.
            speed (float, optional): The speed between 0 and 1. Defaults to 0.3.

        Raises:
            ValueError: If given an unreasonable distance to move.
        """
        # Check if the platform is still moving. Ignore if so. # TODO: Cope better in the long run.
        if self.is_moving():
            logging.warning(
                "The platform is still moving from the last operation. Will ignore this command"
            )
            return

        # Use PI control to determine PWM adjustments
        # TODO
        # heading_adjustment = self.heading_controller.update(delta_theta)

        # Safety checks for steps (assuming maximum safe steps as max_safe_steps)

        if abs(distance) > 100:
            logging.error("Moving way too many revolutions.")
            raise ValueError(
                "move_to_heading: Requested steps exceed the maximum safe range."
            )

        # Record movement - needed if reversing back to origin
        if heading != 0:
            # Set heading
            self.record_movement(MovementType.TURN, heading)
            self.status.moving = True
            self.status.publish()
            left, right = self._calculate_heading(heading)
            self.left.drive_to_dist_relative(left, speed)
            self.right.drive_to_dist_relative(right, speed)
            self.wait_for_movement()

        if distance != 0:
            # Move forward by the given number of revolutions
            self.record_movement(MovementType.MOVE, distance)
            self.status.moving = True
            self.status.publish()
            self.left.drive_to_dist_relative(distance, speed)
            self.right.drive_to_dist_relative(distance, speed)
            self.wait_for_movement()

        # Update odometry after movement
        self.update_odometry()

        self.status.moving = False
        self.status.publish()

    def is_moving(self) -> bool:
        """Checks if the platform is moving.

        Returns:
            bool: True if currently moving.
        """
        return (not self.left.target_reached()) and not (self.right.target_reached())

    def wait_for_movement(self):
        """Waits until the movement operation is complete."""
        while self.is_moving():
            time.sleep(0.1)
        logging.debug(f"Vehicle movement finished")

    def _calculate_heading(self, heading: float) -> Tuple[float, float]:
        """Calculates the linear distance to move each wheel to turn to a specific heading.

        Currently turns from speed center of the axle.

        Takes a heading and converts it to motor distances (left, right).
        """
        dist = heading / math.pi * wheel_base / 2
        return dist, -dist

    # def return_to_origin(self, speed:float=0.5):
    #  """Uses odometry to return to the origin. - use for final """
    #     global x, y, theta
    #     distance_to_origin = math.sqrt(x**2 + y**2)
    #     angle_to_origin = math.atan2(y, x) - theta

    #     # Use PI control to correct heading and distance
    #     self.heading_controller.update(angle_to_origin)
    #     self.distance_controller.update(distance_to_origin / (2 * math.pi * wheel_radius))

    #     self.move_to_heading(angle_to_origin, distance_to_origin / (2 * math.pi * wheel_radius), speed)
    #     self.move_to_heading(-theta, 0, speed)  # Correct orientation

    # this one returns to origin by reversing actions
    def return_to_origin(self, speed: float = 1):
        # return self.return_to_origin_simple(speed)
        return self.return_to_origin_direct(speed)

    def _calculate_origin_move(self, history) -> Tuple[float, float]:
        """Calculates the relative heading and distance to return to the origin."""

        def polar_to_cartesian(heading: float, distance: float) -> Tuple[float, float]:
            """Converts polar coordinates into cartesian representation.

            Args:
                heading (float): The heading in radians.
                distance (float): The distance in m.

            Returns:
                Tuple[float, float]: X and Y coordinates in m.
            """
            x = distance * math.cos(heading)
            y = distance * math.sin(heading)
            return x, y

        def cartesian_to_polar(x: float, y: float) -> Tuple[float, float]:
            """Converts cartesian coordinates pack into polar representation.

            Args:
                x (float): The x position in m.
                y (float): The y position in m.

            Returns:
                Tuple[float, float]: The heading in radians and the distance in m.
            """
            # Calculate the heading (working from -pi to pi currently).
            # I know this can be simplified based on signs of each function, but I can't be bothered :).
            small_angle = math.atan(abs(y / x))
            angle = 0
            if x >= 0 and y >= 0:
                # Quadrant 1
                angle = small_angle
            elif x < 0 and y >= 0:
                # Quadrant 2
                angle = math.pi - small_angle
            elif x < 0 and y < 0:
                # Quadrant 3
                angle = -math.pi + small_angle
            elif x >= 0 and y < 0:
                # Quadrant 4
                angle = -small_angle

            # Calculate the distance
            distance = math.sqrt(x**2 + y**2)

            return angle, distance

        # State variables that acumulate movements
        x_move = 0
        y_move = 0
        heading = math.pi  # Turn around as the first step.

        # Process and sum movements
        for i in range(len(history) - 1, -1, -1):
            movement = history[i]
            logging.debug(
                f"{x_move=}, {y_move=}, {heading=}, About to reverse {i=} {movement}"
            )
            if movement.type == MovementType.MOVE:
                # Move this distance in the current heading.
                x_change, y_change = polar_to_cartesian(heading, movement.value)
                x_move += x_change
                y_move += y_change
            elif movement.type == MovementType.TURN:
                # Adjust the heading for the next movement.
                heading -= movement.value  # Opposite angles.

        # X and Y have the summed positions. Let's convert this back to polar and send it.
        new_head, new_dist = cartesian_to_polar(x_move, y_move)
        return new_head, new_dist

    def return_to_origin_direct(self, speed: float = 1, max_dist=0.5):
        """Calculates the angle to face directly at the home position and drive directly to it.

        Args:
            speed (float, optional): The speed to drive at. Defaults to 0.5.
            max_dist (float, optional) The maximum distance to drive at a time.
        """
        logging.info("Returning directly to origin")
        new_head, new_dist = self._calculate_origin_move(self.movement_history)

        # Move back. Perform rotation and first leg of journey.
        logging.debug(
            f"Relative home position is {x=}, {y=} ({new_head=}, {new_dist=})"
        )
        if new_dist > max_dist:
            # The distance is too big to cover in one go. One of the motors runs slower than the other, so the dodgy way to fix this is to use many short hops to allow the direction / encoders to be mostly corrected at the end of each hop.
            logging.debug(f"Moving back in multiple steps")
            self.move_to_heading(new_head, max_dist, speed)
            new_dist -= max_dist
            self.wait_for_movement()

            # Move the rest of the way
            while new_dist > 0:
                self.move_to_heading(0, max_dist, speed)
                new_dist -= max_dist
                self.wait_for_movement()
        else:
            logging.debug(f"Moving back in one go")
            self.move_to_heading(new_head, new_dist, speed)
            self.wait_for_movement()

        # Rotate back to the original orientation
        logging.debug("Rotating to the original orientation")
        # We are still going forwards when going back to origin, so steps in the motors are still incrementing. This means we can't use `self.right.drive_to_dist(0)``
        self.move_to_heading(-new_head, 0)
        self.wait_for_movement()

        logging.debug(f"Done returning to home.")
        self.update_odometry()

    def return_to_origin_simple(self, speed: float = 0.5):
        """Reverse all previous movements to return to the origin."""
        logging.info("Returning to origin")

        # Process movements in reverse order
        while self.movement_history:
            movement = self.movement_history.pop()
            logging.debug(f"Reversing movement {movement}")
            if movement.type == MovementType.MOVE:
                # Reverse forward movement
                self.left.drive_to_dist_relative(-movement.value, speed)
                self.right.drive_to_dist_relative(-movement.value, speed)
                self.wait_for_movement()
            elif movement.type == MovementType.TURN:
                # Reverse turn movement
                left, right = self._calculate_heading(movement.value)
                self.left.drive_to_dist_relative(-left, speed)
                self.right.drive_to_dist_relative(-right, speed)
                self.wait_for_movement()

        # # Ensure the robot is correctly oriented
        # TODO
        # self.left.drive_to_angle_relative(-theta, speed)
        # self.right.drive_to_angle_relative(theta, speed)
        # self.wait_for_movement()

        # Clear movement history to avoid re-execution
        # self.movement_history.clear()

        # Update odometry after movement
        self.update_odometry()


# Callback functions for MQTT topics
def parse_and_move(arg):
    """Example of published message on topic of robot/move_to_heading with payload:
    {
    "heading": 90,
    "revolutions": 2
    }

    - This suscribes topic robot/move_to_heading
    - receives the message, triggering move_to_heading_callback
    - callback parses the payload and instructs robot to turn a 90-degree heading and move forwards by 2 revolutions
    """
    payload = arg
    heading = payload.get("heading", 0)
    revolutions = payload.get("distance", 0)

    # Print the received message and parsed values
    # print(f"parse_and_move: Received message: {message.payload}")
    logging.info(f"parse_and_move: Parsed heading: {heading}")
    logging.info(f"parse_and_move: Parsed revolutions: {revolutions}")

    vehicle.move_to_heading(heading, revolutions, 0.3)


def parse_and_return(arg):
    """Returns back to the starting point.

    Args:
        arg (JSON object): Currently unused.
    """
    vehicle.move_to_heading(0, -0.2, 0.3)
    vehicle.return_to_origin(0.4)


# def move_to_heading_callback(arg):
#     parse_and_move(client, message, vehicle.move_to_heading)

# def return_to_origin_callback(client, userdata, message):
#     vehicle.return_to_origin()


# Function to encapsulate the MQTT setup
def initialize_mqtt(vehicle: Vehicle):
    """Initializes MQTT and sets up the topic subscriptions and callbacks."""

    # Define callback methods paired with their corresponding MQTT topics
    method_pairs = [
        TopicMethodPair(MQTTTopics.ODOMETRY_MOVE, parse_and_move),
        TopicMethodPair(MQTTTopics.ODOMETRY_GO_HOME, parse_and_return),
    ]

    # Setup MQTT with the defined topic-method pairs
    setup_mqtt(method_pairs)

    # Start the MQTT client loop (use loop_start() if you want non-blocking behavior)
    mqtt_client.loop_forever()  # Use mqtt_client.loop_start() to not block.


if __name__ == "__main__":
    # Main code to run.
    setup_logging("log_odometry.txt", logging.DEBUG)

    # Create the vehicle instance
    vehicle = Vehicle()

    # Initialize MQTT with the vehicle instance
    initialize_mqtt(vehicle)
