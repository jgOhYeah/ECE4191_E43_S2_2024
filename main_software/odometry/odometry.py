#!/usr/bin/env python3
"""Uses the rotatry encoders and any other sensor data to calculate current position, velocity and heading.
Waits for MQTT message to move and actions it."""
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

# Import necessary modules for odometry
from typing import List
import sys
import json
import logging
import time
import math
from gpiozero import Motor, RotaryEncoder
import threading

# Global variables for odometry
x = 0.0
y = 0.0
theta = 0.0
left_count = 0
right_count = 0
wheel_radius = 0.027  # in meters
wheel_base = 0.20  # distance between wheels in meters
ticks_per_revolution = 48 # 

class PIController:
    def __init__(self, kp, ki, sample_time):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.sample_time = sample_time  # Time between control updates
        self.integral = 0
        self.previous_error = 0

    def update(self, error):
        self.integral += error * self.sample_time
        proportional = self.kp * error
        integral = self.ki * self.integral
        output = proportional + integral
        self.previous_error = error
        return output

class Side:
    def __init__(self, motor_a:int, motor_b:int, motor_en:int, enc_a:int, enc_b:int, name:str=""):
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
        self.direction = 1
        self.target_steps = 0
        self.movement_complete = False  # Flag to indicate movement completion
        self.encoder.when_rotated = self._are_we_there_yet

    def drive_to_steps(self, steps:int, speed:float=1):
        """Drives the motor to a given number of steps.

        Args:
            steps (int): The number of steps (absolute, not relative).
            speed (float, optional): How fast to move. Defaults to 1.
        """
        print(f"drive_to_steps: {self.name} driving to {steps} ({self._steps_to_angle(steps)} radians)")
        self.direction = 1 # Allows comparisons in the opposite direction if needed.
        self.target_steps = steps

        # TODO: Soft start and stop.
        if steps > self.encoder.steps:
            # Need to go forwards
            self.motor.forward(speed)
        else:
            # Need to go backwards
            self.motor.backward(speed)
            self.direction = -1


    def drive_to_angle(self, angle:float, speed:float=1):
        """Drives the motor to a given absolute angle.

        Args:
            angle (float): The number of radians (absolute, not relative).
            speed (float, optional): How fast to move. Defaults to 1.
        """

        self.drive_to_steps(self._angle_to_steps(angle), speed)

    def drive_to_angle_relative(self, angle:float, speed:float=1):
        """Drives the motor to a given angle relative to the current position.

        Args:
            angle (float): The number of radians (relative, not absolute).
            speed (float, optional): How fast to move. Defaults to 1.
        """
        abs_steps = self._angle_to_steps(angle) + self.encoder.steps
        self.drive_to_steps(abs_steps, speed)

    def _angle_to_steps(self, angle:float) -> int:
        return int(angle/(2*math.pi) * ticks_per_revolution)
    
    def _steps_to_angle(self, steps:int) -> float:
        return steps / ticks_per_revolution * 2 * math.pi
    
    def _are_we_there_yet(self):
        distance_to_go = self.direction * (self.target_steps - self.encoder.steps)
        if distance_to_go < 0:
            # We got there. Stop
            self.motor.stop()
            print("_are_we_there_yet: Motor stopped")
            # TODO: Callback or something for when the motor is stopped.
            self.movement_complete = True  # Indicate that the movement is complete

    def target_reached(self) -> bool:
        #check if the motors have reached their target
        return self.movement_complete

class Vehicle:
    def __init__(self):
        self.left = Side(5, 6, 13, 19, 26, "left")
        self.right = Side(7, 16, 12, 20, 21, "right")

        self.odometry_thread = threading.Thread(target=self.odometry_loop)
        self.odometry_thread.daemon = True  # Daemon thread exits when the main program exits
        self.odometry_thread.start()
        
        # Initialize PI controllers for heading and distance control
        self.heading_controller = PIController(kp=1.0, ki=0.1, sample_time=0.1)
        self.distance_controller = PIController(kp=1.0, ki=0.1, sample_time=0.1)

        # Initialize movement history stack - needed if returning to origin by reversing
        self.movement_history = []

    def record_movement(self, action, value):
        """Records a movement action and its value - needed if returning to origin by reversing. """
        self.movement_history.append((action, value))
    
    def odometry_loop(self):
        while True:
            self.update_odometry()
            time.sleep(0.1)  # Adjust the sleep duration as necessary
    
    def update_odometry(self):
        global x, y, theta, left_count, right_count
        print("update_odometry: self.left.encoder.steps", self.left.encoder.steps)
        print("update_odometry: self.right.encoder.steps", self.right.encoder.steps)
        left_distance = (2 * math.pi * wheel_radius * self.left.encoder.steps) / ticks_per_revolution
        right_distance = (2 * math.pi * wheel_radius * self.right.encoder.steps) / ticks_per_revolution
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / wheel_base
        x += distance * math.cos(theta + delta_theta / 2.0)
        y += distance * math.sin(theta + delta_theta / 2.0)
        theta += delta_theta
        logger.debug(f"update_odometry: Position: x={x:.2f}, y={y:.2f}, theta={theta:.2f} radians")
    
    def move_to_heading(self, heading: float, revolutions: int, speed: float = 0.5):
        global theta

        # Normalize the heading to the range [-π, π]
        delta_theta = heading - theta
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi

        # Use PI control to determine PWM adjustments
        heading_adjustment = self.heading_controller.update(delta_theta)

        # Safety checks for steps (assuming maximum safe steps as max_safe_steps)
        max_safe_steps = ticks_per_revolution * 10  # Define a safe limit for revolutions
        steps = int(revolutions * ticks_per_revolution)
        if abs(steps) > max_safe_steps:
            raise ValueError("move_to_heading: Requested steps exceed the maximum safe range.")
        
        # Record movement - needed if reversing back to origin 
        if revolutions != 0:
            self.record_movement('move_to_heading: forward', revolutions)
        if delta_theta != 0:
            self.record_movement('move_to_heading: turn', delta_theta)

        # Set PWM values based on PI control outputs
        self.left.motor.forward(speed + heading_adjustment)
        self.right.motor.forward(speed - heading_adjustment)

        # Move to the desired heading
        self.left.drive_to_angle_relative(delta_theta, speed)
        self.right.drive_to_angle_relative(-delta_theta, speed)
        self.wait_for_movement()

        # Move forward by the given number of revolutions
        self.left.drive_to_steps(steps, speed)
        self.right.drive_to_steps(steps, speed)
        self.wait_for_movement()

        # Update odometry after movement
        self.update_odometry()
        
    def wait_for_movement(self):
        while not (self.left.target_reached() and self.right.target_reached()):
            time.sleep(0.1)

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
    def return_to_origin(self, speed: float = 0.5):
        """Reverse all previous movements to return to the origin."""
        global theta

        # Process movements in reverse order
        while self.movement_history:
            action, value = self.movement_history.pop()
            if action == 'forward':
                # Reverse forward movement
                self.left.drive_to_steps(-value * ticks_per_revolution, speed)
                self.right.drive_to_steps(-value * ticks_per_revolution, speed)
                self.wait_for_movement()
            elif action == 'turn':
                # Reverse turn movement
                self.left.drive_to_angle_relative(-value, speed)
                self.right.drive_to_angle_relative(value, speed)
                self.wait_for_movement()

        # Ensure the robot is correctly oriented
        self.left.drive_to_angle_relative(-theta, speed)
        self.right.drive_to_angle_relative(theta, speed)
        self.wait_for_movement()

        # Clear movement history to avoid re-execution
        self.movement_history.clear()

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
    revolutions = payload.get("revolutions", 0)
    vehicle.move_to_heading(heading, revolutions)

    # Print the received message and parsed values
    # print(f"parse_and_move: Received message: {message.payload}")
    print(f"parse_and_move: Parsed heading: {heading}")
    print(f"parse_and_move: Parsed revolutions: {revolutions}")

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
        TopicMethodPair(MQTTTopics.ODOMETRY_GO_HOME, vehicle.return_to_origin),
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
