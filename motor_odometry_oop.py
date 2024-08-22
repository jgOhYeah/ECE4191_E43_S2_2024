#!/usr/bin/env python3
from gpiozero import Motor, RotaryEncoder
import time
import math

wheel_radius = 0.027  # in meters
wheel_base = 0.20  # distance between wheels in meters
ticks_per_revolution = 30*48 # 47*48

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
        self.encoder.when_rotated = self._are_we_there_yet

    def drive_to_steps(self, steps:int, speed:float=1):
        """Drives the motor to a given number of steps.

        Args:
            steps (int): The number of steps (absolute, not relative).
            speed (float, optional): How fast to move. Defaults to 1.
        """
        print(f"{self.name} driving to {steps} ({self._steps_to_angle(steps)} radians)")
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

    def current_angle(self) -> float:
        """Converts the number of steps to radians.

        Returns:
            float: The wheel angle in radians.
        """
        return self._steps_to_angle(self.encoder.steps)

    def _angle_to_steps(self, angle:float) -> int:
        return int(angle/(2*math.pi) * ticks_per_revolution)
    
    def _steps_to_angle(self, steps:int) -> float:
        return steps / ticks_per_revolution * 2 * math.pi
    
    def _are_we_there_yet(self):
        distance_to_go = self.direction * (self.target_steps - self.encoder.steps)
        if distance_to_go < 0:
            # We got there. Stop
            self.motor.stop()
            # print("Motor stopped")
            # TODO: Callback or something for when the motor is stopped.

class Vehicle:
    def __init__(self):
        self.left = Side(5, 6, 13, 19, 26, "left")
        self.right = Side(7, 16, 12, 20, 21, "right")
    
    def demo(self, speed=0.2):
        print("Demo")
        # print("Forwards 1")
        self.left.drive_to_angle_relative(5*2*math.pi, speed)
        self.right.drive_to_angle_relative(5*2*math.pi, speed)
        # time.sleep(10)

        # print("Turn 1")
        # self.left.drive_to_angle_relative(3*2*math.pi, speed)
        # self.left.drive_to_angle_relative(-3*2*math.pi, speed)
        # time.sleep(10)

        # print("Forwards 3")
        # self.left.drive_to_angle_relative(5*2*math.pi, speed)
        # self.right.drive_to_angle_relative(5*2*math.pi, speed)
        # time.sleep(10)

        # print("Backwards 4")
        # self.left.drive_to_angle_relative(-5*2*math.pi, speed)
        # self.right.drive_to_angle_relative(-5*2*math.pi, speed)
        # time.sleep(10)

        # print("Turn 5")
        # self.left.drive_to_angle_relative(-3*2*math.pi, speed)
        # self.right.drive_to_angle_relative(3*2*math.pi, speed)
        # time.sleep(10)

        # print("Backwards 6")
        # self.left.drive_to_angle_relative(-5*2*math.pi, speed)
        # self.right.drive_to_angle_relative(-5*2*math.pi, speed)
        # time.sleep(10)

        # print("Done")
        # time.sleep(15)

        while True:
            if self.left._are_we_there_yet() and self.right._are_we_there_yet():
                break
            time.sleep(0.1)


if __name__ == "__main__":
    vehicle = Vehicle()
    while True:
        vehicle.demo(0.5)