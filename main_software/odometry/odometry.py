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
    MoveSpeed
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

from motor_control import MotorAccelerationController, MotorSpeedController, MotorPositionController, PIControllerLogged
from digitalfilter import create_filter
from kalman import PositionAccumulator, PositionAccumulatorLogged

# Constants
WHEEL_RADIUS = 0.027  # in meters
WHEEL_BASE = 3.05 * 0.222  # distance between wheels in meters
TICKS_PER_REVOLUTION = 19 * 47  #
MAX_MOTOR_SPEED = 1700 # Max speed in ticks per second.

class Side:
    """Class for controlling the motor and encoder on a side."""

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
        motor = Motor(motor_a, motor_b, enable=motor_en, pwm=True)
        encoder = RotaryEncoder(enc_a, enc_b, max_steps=0)
        self.name = name

        # Controller and its constants
        accel_kp = 0.00001
        accel_ki = 0.001
        accel_windup = 20
        accel_on = 5000
        accel_off = 20000
        accel_control = MotorAccelerationController(
            PIControllerLogged(accel_kp, accel_ki, accel_windup, name, "acceleration.csv"),
            motor,
            encoder,
            accel_on,
            accel_off,
            create_filter(4, 20, 1/0.02)
        )

        # Create the speed controller
        speed_kp = 8
        speed_ki = 0.5
        speed_windup = 3
        speed_control = MotorSpeedController(
            PIControllerLogged(speed_kp, speed_ki, speed_windup, name, "speed.csv"),
            accel_control,
            create_filter(4, 20, 1/0.02)
        )
        # test_acceleration(accel_control, speed_control)
        # test_speed(speed_control)

        pos_kp = 2
        pos_ki = 0.2
        pos_windup = 0.00

        self.pos_control = MotorPositionController(
            PIControllerLogged(pos_kp, pos_ki, pos_windup, name, "position.csv"),
            speed_control
        )

    def _drive_to_steps(self, steps: int, speed: int = 100):
        """Drives the motor to a given number of steps.

        Args:
            steps (int): The number of steps (absolute, not relative).
            speed (int, optional): How fast to move in steps / second. Defaults to 100.
        """
        self.pos_control.set_target_position(steps, speed)

    def _drive_to_angle(self, angle: float, speed: int = 100):
        """Drives the motor to a given absolute angle.

        Args:
            angle (float): The number of radians (absolute, not relative).
            speed (int, optional): How fast to move. Defaults to 100.
        """

        self._drive_to_steps(self._angle_to_steps(angle), speed)

    def _drive_to_angle_relative(self, angle: float, speed: int = 100):
        """Drives the motor to a given angle relative to the current position.

        Args:
            angle (float): The number of radians (relative, not absolute).
            speed (int, optional): How fast to move. Defaults to 100.
        """
        abs_steps = (
            self._angle_to_steps(angle) + self.pos_control.position_control.target
        )  # self.target_steps assumes that the last target was reached, self.encoder.steps accrues errors.
        self._drive_to_steps(abs_steps, speed)

    def drive_to_dist(self, dist: float, speed: int = 100):
        """Drives the motor to a given absolute distance.

        Args:
            distance (float): The number of m (absolute, not relative).
            speed (int, optional): How fast to move. Defaults to 100.
        """
        self._drive_to_angle(self._dist_to_angle(dist), speed)

    def drive_to_dist_relative(self, dist: float, speed: int = 100):
        """Drives the motor to a given distance relative to the current position.

        Args:
            distance (float): The number of m (relative, not absolute).
            speed (int, optional): How fast to move. Defaults to 100.
        """
        self._drive_to_angle_relative(self._dist_to_angle(dist), speed)

    def _dist_to_angle(self, dist: float) -> float:
        return dist / WHEEL_RADIUS

    def _angle_to_steps(self, angle: float) -> int:
        return int(angle / (2 * math.pi) * TICKS_PER_REVOLUTION)

    def _steps_to_angle(self, steps: int) -> float:
        return steps / TICKS_PER_REVOLUTION * 2 * math.pi

    def _angle_to_dist(self, angle: float) -> float:
        return angle * WHEEL_RADIUS

    def pos_target_reached(self) -> bool:
        """Checks if the motors have reached their target.

        Returns:
            bool: Are we there yet?
        """
        return self.pos_control.pos_movement_complete()
        # return self.movement_complete

    def stop(self):
        """Stops the motor."""
        self.pos_control.set_target_speed(0)
    
    def set_speed(self, speed:float) -> None:
        """Sets the motor to a given speed in m/s.

        Limits the maximum speed to 2000 steps / second as that is near the motor's top speed.

        Args:
            speed (float): The speed in m/s.
        """
        speed = self._angle_to_steps(self._dist_to_angle(speed))
        if speed > MAX_MOTOR_SPEED:
            logging.warning(f"Max motor speed of {MAX_MOTOR_SPEED} steps/s reached, will not go at requested {speed} steps/s.")
            speed = MAX_MOTOR_SPEED

        self.pos_control.set_target_speed(speed)

    def get_speed(self) -> float:
        """Gets the current speed of the motor in m/s.

        Returns:
            float: The current speed in metres / second.
        """
        return self._angle_to_dist(self._steps_to_angle(self.pos_control.get_speed()))

    def update(self) -> None:
        """Updates the controllers.
        """
        self.pos_control.update()


class Vehicle:
    PUBLISH_INTERVAL = 8 # Only publish status updates every few iterations.
    UPDATE_DELAY = 0.02

    def __init__(self):
        """Initialises the vehicle and starts the controllers.
        """
        # Setup each side.
        self.left = Side(6, 5, 13, 19, 26, "left")
        self.right = Side(16, 7, 12, 20, 21, "right")

        # Position and velocity.
        self.position = PositionAccumulatorLogged()
        self.angular_velocity, self.rotation_centre, self.linear_velocity = 0, 0, 0
        self.last_update = time.time()

        # Start the motor control threads.
        self.publish_count = Vehicle.PUBLISH_INTERVAL
        thread = threading.Thread(target=self.run, args=())
        thread.start()

    def _calculate_velocity(
        self, left_speed: float, right_speed: float
    ) -> Tuple[float, float, float]:
        """Calculates the angular velocity, center of rotation relative to the middle of the robot and tangential
        velocity based on the left and right wheel speeds.

        Args:
            left_speed (float): The left speed in m/s.
            right_speed (float): The right speed in m/s.

        Returns:
            Tuple[float, float, float]: The angular velocity (clockwise positive), the distance of the center of
                                        rotation from the axle center (right is positive) and the linear velocity
                                        (forwards is positive).
        """
        # See the wheel_calculations jupyter notebook for the derivation of these functions.
        def angular_velocity(c1, c2, aw):
            return -(c2 - c1) / aw

        def centre_rad(c1, c2, aw):
            if c1 == c2:
                return math.inf
            else:
                return aw * (-c1 - c2) / (2 * (c1 - c2))

        return (
            angular_velocity(left_speed, right_speed, WHEEL_BASE),
            centre_rad(left_speed, right_speed, WHEEL_BASE),
            (left_speed + right_speed) / 2
        )

    def _calculate_speeds(self, angular_velocity:float, centre_rad:float, speed:float) -> Tuple[float, float]:
        """Calculates the left and right motor speeds required.

        Args:
            angular_velocity (float): _description_
            centre_rad (float): _description_
            speed (float): _description_

        Returns:
            Tuple[float, float]: Left and right speeds in m/s respectively.
        """
        angular_contribution = WHEEL_BASE * angular_velocity / 2
        return [
            -angular_contribution + speed,
            angular_contribution + speed
        ]

    def predict(self, angular_velocity:float, centre_radius:float, tangential_speed:float, timestep:float) -> Tuple[float, float, float]:
        """Predicts the left-right change, forwards change and new angle.

        Args:
            angular_velocity (float): The angular velocity of the robot rotating.
            centre_rad (float): The centre of rotation. 0 is between the two wheels.
            speed (float): The tangential speed.
            timestep (float): The time step.

        Returns:
            Tuple[float, float, float]: The left-right change, forwards-backward change and heading change.
        """
        # See the jupyter notebook.
        angle_change = angular_velocity * timestep
        if angle_change != 0:
            # Going around a corner.
            forwards = centre_radius * math.sin(angle_change)
            sideways = centre_radius - centre_radius * math.cos(angle_change)
        else:
            # Going straight forwards or backwards.
            forwards = tangential_speed * timestep
            sideways = 0

        return forwards, sideways, angle_change
    
    def update(self) -> None:
        """Updates the vehicle state.
        """
        # Calculate the time step.
        timestamp = time.time()
        timestep = timestamp - self.last_update
        self.last_update = timestamp

        # Update the controllers.
        self.left.update()
        self.right.update()

        # Calculate the change in vehicle position.
        self.angular_velocity, self.rotation_centre, self.linear_velocity = self._calculate_velocity(self.left.get_speed(), self.right.get_speed())
        forwards_change, sideways_change, angle_change = self.predict(self.angular_velocity, self.rotation_centre, self.linear_velocity, timestep)
        self.position.add_relative(forwards_change, sideways_change, angle_change)
        self.position.add_current(self.angular_velocity, self.rotation_centre, self.linear_velocity)
        
        # Report the current position if needed
        self.publish_count += 1
        if self.publish_count >= Vehicle.PUBLISH_INTERVAL:
            self.publish_count = 0
            self._publish_status()

    def run(self) -> None:
        """Calls update continuously in a loop."""
        while True:
            self.update()
            time.sleep(Vehicle.UPDATE_DELAY)

    def _publish_status(self) -> None:
        """Publishes the current status to MQTT
        """
        status = OdometryCurrent(
            heading=self.position.heading,
            position=(self.position.x, self.position.y),
            speed=self.linear_velocity,
            angular_velocity=self.angular_velocity,
            moving=self.is_moving(),
            turn_radius=self.rotation_centre
        )
        status.publish()

    def move_to_heading(self, heading: float, distance: int, speed: float = 100): # XXX: Replace
        """Moves the platform to a specific relative heading and position.

        Args:
            heading (float): The new heading in radians, positive is clockwise when viewed from above.
            distance (int): Distance to move in m.
            speed (float, optional): The speed in steps / second.

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
            # self.record_movement(MovementType.TURN, heading)
            self.status.moving = True
            self.status.publish()
            left, right = self._calculate_heading(heading)
            self.left.drive_to_dist_relative(left, speed)
            self.right.drive_to_dist_relative(right, speed)
            self.wait_for_movement()

        if distance != 0:
            # Move forward by the given number of revolutions
            # self.record_movement(MovementType.MOVE, distance)
            self.status.moving = True
            self.status.publish()
            self.left.drive_to_dist_relative(distance, speed)
            self.right.drive_to_dist_relative(distance, speed)
            self.wait_for_movement()

        self.status.moving = False
        self.status.publish()

    def is_moving(self) -> bool:
        """Checks if the platform is moving.

        Returns:
            bool: True if currently moving.
        """
        # return (not self.left.pos_target_reached()) and not (self.right.pos_target_reached())
        ANGULAR_ERROR = 1e-2
        LINEAR_ERROR = 1e-3
        return abs(self.angular_velocity) < ANGULAR_ERROR and abs(self.linear_velocity) < LINEAR_ERROR

    def wait_for_movement(self):
        """Waits until the movement operation is complete."""
        while self.is_moving():
            time.sleep(0.1)
        logging.debug(f"Vehicle movement finished")

    def _calculate_heading(self, heading: float) -> Tuple[float, float]: # XXX: Replace
        """Calculates the linear distance to move each wheel to turn to a specific heading.

        Currently turns from speed center of the axle.

        Takes a heading and converts it to motor distances (left, right).
        """
        dist = heading / math.pi * WHEEL_BASE / 2
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
    def return_to_origin(self, speed: float = 1): # XXX: Replace
        raise NotImplementedError("Return to origin not implemented")
        # return self.return_to_origin_simple(speed)
        return self.return_to_origin_direct(speed)

    def _calculate_origin_move(self, history) -> Tuple[float, float]: # XXX: Replace
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

    def return_to_origin_direct(self, speed: float = 1, max_dist=0.5):# XXX: Replace
        """Calculates the angle to face directly at the home position and drive directly to it.

        Args:
            speed (float, optional): The speed to drive at. Defaults to 0.5.
            max_dist (float, optional) The maximum distance to drive at a time.
        """
        raise NotImplementedError("Return to origin not implemented")
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

    def return_to_origin_simple(self, speed: float = 0.5):# XXX: Replace
        """Reverse all previous movements to return to the origin."""
        raise NotImplementedError("Return to origin not implemented")
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
    
    def move_speed(self, move_speed:MoveSpeed) -> None:
        """Commands the vehicle to move with a given speed.

        Args:
            move_speed (MoveSpeed): The object containing the command
        """
        logging.debug(f"Got move command, {move_speed}")
        left, right = self._calculate_speeds(move_speed.angular_velocity, move_speed.turn_radius, move_speed.speed)
        self.left.set_speed(left)
        self.right.set_speed(right)

# Function to encapsulate the MQTT setup
def initialize_mqtt(vehicle:Vehicle):
    """Initializes MQTT and sets up the topic subscriptions and callbacks."""

    # Define callback methods paired with their corresponding MQTT topics
    method_pairs = [
        MoveSpeed(callback=vehicle.move_speed).topic_method_pair(),
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
