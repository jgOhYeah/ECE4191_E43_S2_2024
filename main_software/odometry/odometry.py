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
    MoveSpeed,
    MovePosition,
    BallLoadMsg,
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
from gpiozero import Motor, RotaryEncoder, LED
import threading
from dataclasses import dataclass
from enum import Enum
from abc import ABC, abstractmethod
import threading
from sensors import main as sensors_main

from motor_control import (
    MotorAccelerationController,
    MotorSpeedController,
    MotorPositionController,
    PIController,
    PIControllerLogged,
    PIControllerAngle,
    MotorControlMode,
)
from digitalfilter import create_filter
from kalman import PositionAccumulator, PositionAccumulatorLogged
from ball_loading import ball_loader

# Constants
WHEEL_RADIUS = 0.027  # in meters
WHEEL_BASE = 0.222  # distance between wheels in meters
TICKS_PER_REVOLUTION = 19 * 47  #
MAX_MOTOR_SPEED = 1700  # Max speed in ticks per second.


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
            PIControllerLogged(
                accel_kp, accel_ki, accel_windup, name, "acceleration.csv"
            ),
            motor,
            encoder,
            accel_on,
            accel_off,
            create_filter(4, 20, 1 / 0.02),
        )

        # Create the speed controller
        speed_kp = 8
        speed_ki = 0.5
        speed_windup = 3
        speed_control = MotorSpeedController(
            PIControllerLogged(speed_kp, speed_ki, speed_windup, name, "speed.csv"),
            accel_control,
            create_filter(4, 20, 1 / 0.02),
        )
        # test_acceleration(accel_control, speed_control)
        # test_speed(speed_control)

        pos_kp = 2
        pos_ki = 0.2
        pos_windup = 0.00

        self.pos_control = MotorPositionController(
            PIControllerLogged(pos_kp, pos_ki, pos_windup, name, "position.csv"),
            speed_control,
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

    def set_speed(self, speed: float) -> None:
        """Sets the motor to a given speed in m/s.

        Limits the maximum speed to 2000 steps / second as that is near the motor's top speed.

        Args:
            speed (float): The speed in m/s.
        """
        speed = self._angle_to_steps(self._dist_to_angle(speed))
        if speed > MAX_MOTOR_SPEED:
            logging.warning(
                f"Max motor speed of {MAX_MOTOR_SPEED} steps/s reached, will not go at requested {speed} steps/s."
            )
            speed = MAX_MOTOR_SPEED

        self.pos_control.set_target_speed(speed)

    def get_speed(self) -> float:
        """Gets the current speed of the motor in m/s.

        Returns:
            float: The current speed in metres / second.
        """
        return self._angle_to_dist(self._steps_to_angle(self.pos_control.get_speed()))

    def update(self) -> None:
        """Updates the controllers."""
        self.pos_control.update()


class HeadingDistanceControl:
    """Keeps the vehicle pointed in a particular direction and drives to a given distance."""

    def __init__(self):
        self.heading_control = PIControllerAngle(0.01, 0.001, 1, "position", "heading")
        self.heading_control.set_limits(-0.2, 0.2)
        self.distance_control = PIControllerLogged(0.01, 0.001, 1, "position", "speed")
        self.distance_control.set_limits(-0.2, 0.2)
        self.prev_time = time.time()

    def set_speed(self, max_av: float, max_v: float) -> None:
        """Sets the maximum angular and linear velocities."""
        self.heading_control.set_limits(-max_av, max_av)
        self.distance_control.set_limits(-max_v, max_v)

    def set_target(self, heading: float, distance: float) -> None:
        """Sets the target heading and distance.

        Args:
            heading (float): The heading to turn towards.
            distance (float): The distance to go in that heading.
        """
        self.heading_control.set_target(heading)
        self.distance_control.set_target(distance)

    def update(
        self, timestep: float, cur_heading: float, cur_distance: float
    ) -> Tuple[float, float]:
        """Recommends new angular and linear velocities to reach the required distance.

        Args:
            timestep (float): The timestep in seconds.
            cur_heading (float): The current heading.
            cur_distance (float): The current distance.

        Returns:
            Tuple[float, float]: New angular and linear velocities.
        """
        new_angular_velocity = self.heading_control.update(cur_heading, timestep)
        new_linear_velocity = self.distance_control.update(cur_distance, timestep)
        return new_angular_velocity, new_linear_velocity


class PositionControl:
    """Drives the robot to a particular location relative to the start."""

    def __init__(self):
        self.heading_distance = HeadingDistanceControl()
        self.target = (0, 0)
        self.is_relative = False  # Flag to indicate if the target is relative or global

    def set_target(
        self,
        coords: Tuple[float, float],
        cur_pos: Tuple[float, float],
        cur_heading: float,
        is_relative: bool = False,
    ) -> None:
        """
        Sets the target position.

        Args:
            coords (Tuple[float, float]): Target position (x, y).
            is_relative (bool): Whether the target is relative to the current position or global.
        """
        self.target = coords
        self.is_relative = is_relative
        if self.is_relative:
            # Convert relative target to global target based on current position and heading
            self.target = self.calculate_relative_target(cur_pos, cur_heading)
        else:
            # Use the target as a global position
            self.target = self.target

    def set_speed(self, max_av: float, max_v: float) -> None:
        self.heading_distance.set_speed(max_av, max_v)

    @classmethod
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

    @classmethod
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

    @classmethod
    def calculate_target_move(
        current: Tuple[float, float], target: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Calculates a target heading and distance based off where we are now and where we want to go.

        Args:
            current (Tuple[float, float]): Current x and y coordinates.
            target (Tuple[float, float]): Target x and y coordinates.

        Returns:
            Tuple[float, float]: The heading to aim for relative to the start position and how far to go.
        """
        x_change = target[0] - current[0]
        y_change = target[1] - current[1]
        target_heading, target_distance = PositionControl.cartesian_to_polar(
            x_change, y_change
        )
        return target_heading, target_distance

    def calculate_relative_target(
        self, current_pos: Tuple[float, float], cur_heading: float
    ) -> Tuple[float, float]:
        """
        Converts a relative target (with respect to current position and heading) to a global target.

        Args:
            current_pos (Tuple[float, float]): The current global (x, y) position.
            cur_heading (float): The current heading in radians.

        Returns:
            Tuple[float, float]: The global target position.
        """
        # Convert relative target into global coordinates by adjusting for current heading
        relative_x, relative_y = self.target
        delta_x = relative_x * math.cos(cur_heading) - relative_y * math.sin(
            cur_heading
        )
        delta_y = relative_x * math.sin(cur_heading) + relative_y * math.cos(
            cur_heading
        )

        # Add the relative target to the current position
        global_x = current_pos[0] + delta_x
        global_y = current_pos[1] + delta_y

        return global_x, global_y

    def update(
        self, timestep: float, cur_pos: Tuple[float, float], cur_heading: float
    ) -> Tuple[float, float]:
        """Runs the controller and predicts new velocities for each wheel.

        Args:
            timestep (float): The timestep from the last update.
            cur_pos (Tuple[float, float]): The current position relative to the start.
            cur_heading (float): The current heading.

        Returns:
            Tuple[float, float]: The new angular and linear velocities.
        """

        # Calculate the target heading and distance from where we are to the target.
        target_heading, target_distance = PositionControl.calculate_target_move(
            cur_pos, self.target
        )

        # If we are facing the wrong direction, don't bother moving forwards until we are facing the correct way.
        if (
            abs(
                self.heading_distance.heading_control.error(cur_heading, target_heading)
            )
            > math.pi / 4
        ):
            # Facing the wrong way, make linear velocity 0.
            self.heading_distance.set_target(target_heading, target_distance)
        else:
            # Can move forward.
            self.heading_distance.set_target(
                target_heading, 0
            )  # We wish to reach the spot. This could also be set to stop a little before it maybe?

        # Calculate the required angular and linear velocities to reach the target.
        av, lv = self.heading_distance.update(timestep, cur_heading, target_distance)
        return av, lv

    def reset(self) -> None:
        """Resets all history."""
        self.heading_distance.heading_control.reset()
        self.heading_distance.distance_control.reset()


class Vehicle:
    PUBLISH_INTERVAL = 8  # Only publish status updates every few iterations.
    UPDATE_DELAY = 0.02

    def __init__(self):
        """Initialises the vehicle and starts the controllers."""
        # Setup each side.
        self.left = Side(6, 5, 13, 19, 26, "left")
        self.right = Side(16, 7, 12, 20, 21, "right")

        # Position and velocity.
        self.position = PositionAccumulatorLogged()
        self.angular_velocity, self.rotation_centre, self.linear_velocity = 0, 0, 0
        self.last_update = time.time()
        self.control_mode = MotorControlMode.SPEED
        self.position_controller = PositionControl()
        self.last_position_update_time = time.time()
        self.last_position_update_count = 5

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
            (left_speed + right_speed) / 2,
        )

    def _calculate_required_velocity(
        self, angular_velocity: float, speed: float
    ) -> Tuple[float, float]:
        """Calculates the left and right motor speeds required.

        Args:
            angular_velocity (float): _description_
            speed (float): _description_

        Returns:
            Tuple[float, float]: Left and right speeds in m/s respectively.
        """
        angular_contribution = WHEEL_BASE * angular_velocity / 2
        return [-angular_contribution + speed, angular_contribution + speed]

    def predict(
        self,
        angular_velocity: float,
        centre_radius: float,
        tangential_speed: float,
        timestep: float,
    ) -> Tuple[float, float, float]:
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
        """Updates the vehicle state."""
        # Calculate the time step.
        timestamp = time.time()
        timestep = timestamp - self.last_update
        self.last_update = timestamp

        # Calculate the change in vehicle position.
        self.angular_velocity, self.rotation_centre, self.linear_velocity = (
            self._calculate_velocity(self.left.get_speed(), self.right.get_speed())
        )
        forwards_change, sideways_change, angle_change = self.predict(
            self.angular_velocity, self.rotation_centre, self.linear_velocity, timestep
        )
        self.position.add_relative(forwards_change, sideways_change, angle_change)
        self.position.add_current(
            self.angular_velocity, self.rotation_centre, self.linear_velocity
        )

        # Update tbe heading and direction controllers as needed. Only do this every so often so that the other controllers have time to work.
        self.last_position_update_count += 1
        if (
            self.control_mode == MotorControlMode.POSITION
            and self.last_position_update_count > 5
        ):
            self.last_position_update_count = 0
            pos_timestep = time.time() - self.last_position_update_time
            recommended_av, recommended_v = self.position_controller.update(
                pos_timestep, self.position.as_tuple(), self.position.heading
            )
            self._move_speed(recommended_av, recommended_v)

        # Update the controllers.
        self.left.update()
        self.right.update()

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
        """Publishes the current status to MQTT"""
        status = OdometryCurrent(
            heading=self.position.heading,
            position=(self.position.x, self.position.y),
            speed=self.linear_velocity,
            angular_velocity=self.angular_velocity,
            moving=self.is_moving(),
            turn_radius=self.rotation_centre,
        )
        status.publish()

    def is_moving(self) -> bool:
        """Checks if the platform is moving.

        Returns:
            bool: True if currently moving.
        """
        # return (not self.left.pos_target_reached()) and not (self.right.pos_target_reached())
        ANGULAR_ERROR = 1e-2
        LINEAR_ERROR = 1e-3
        return not (
            abs(self.angular_velocity) < ANGULAR_ERROR
            and abs(self.linear_velocity) < LINEAR_ERROR
        )

    def _move_speed(self, angular_velocity: float, linear_velocity: float) -> None:
        """Commands the vehicle to move with a given speed.

        This is for internal use only. mqtt_move_speed correctly sets the
        vehicle mode and disables position control if enabled.
        """
        left, right = self._calculate_required_velocity(
            angular_velocity, linear_velocity
        )
        self.left.set_speed(left)
        self.right.set_speed(right)

    def mqtt_move_speed(self, move_speed: MoveSpeed) -> None:
        """Commands the vehicle to move with a given speed.

        Args:
            move_speed (MoveSpeed): The object containing the command
        """
        logging.debug(f"Got move command, {move_speed}")
        self.control_mode = MotorControlMode.SPEED
        self._move_speed(move_speed.angular_velocity, move_speed.speed)

    def mqtt_move_position(
        self, move_position: MovePosition, is_relative: bool = True
    ) -> None:
        """Commands the vehicle to move to a specific position."""
        logging.debug(f"Got move to position command {move_position}")
        if self.control_mode == MotorControlMode.SPEED:
            # Changing from speed control to position control. Reset the history.
            self.position_controller.reset()
            self.control_mode = MotorControlMode.POSITION

        self.position_controller.set_speed(
            move_position.angular_velocity, move_position.speed
        )

        current_position = self.position.as_tuple()
        current_heading = self.position.heading

        self.position_controller.set_target(
            move_position.position,
            current_position,
            current_heading,
            is_relative=is_relative,
        )
        self.last_position_update_count = 1000  # Force a new update. # TODO: Mutexes


class LEDFlash:
    def __init__(self, pin: int):
        """Flashes an LED on callback.

        Args:
            pin (int): _description_
        """
        self.led = LED(pin)
        self.led.on()

    def get_topic_pair(self) -> TopicMethodPair:
        return TopicMethodPair("#", self.flash)

    def flash(self, args) -> None:
        print("Hi")
        logging.info("Flash")
        self.led.blink(0.01, 1, None)
        self.led.on()

led = LEDFlash(17)
# Function to encapsulate the MQTT setup
def initialize_mqtt(vehicle: Vehicle):
    """Initializes MQTT and sets up the topic subscriptions and callbacks."""

    # Define callback methods paired with their corresponding MQTT topics
    method_pairs = [
        # led.get_topic_pair(),
        MoveSpeed(callback=vehicle.mqtt_move_speed).topic_method_pair(),
        MovePosition(callback=vehicle.mqtt_move_position).topic_method_pair(),
        BallLoadMsg(callback=ball_loader.receive_start).topic_method_pair(),
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

    # new thread for sensors
    t = threading.Thread(target=sensors_main)  ########
    t.start

    # Initialize MQTT with the vehicle instance
    initialize_mqtt(vehicle)
