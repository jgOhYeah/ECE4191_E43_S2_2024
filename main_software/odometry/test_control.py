"""test_control.py
Tests the motor controller."""

import gpiozero
from gpiozero import Motor, RotaryEncoder
from gpiozero.pins.mock import MockFactory, MockPWMPin

gpiozero.Device.pin_factory = MockFactory(pin_class=MockPWMPin)

from motor_control import MotorSpeedController, MotorPositionController

import time
import logging
import sys

sys.path.insert(1, sys.path[0] + "/../")
from defines import setup_logging

if __name__ == "__main__":
    setup_logging()
    logging.info("Control test started")
    # Create the motor objects.
    motor_a = 0
    motor_b = 1
    motor_en = 2
    enc_a = 3
    enc_b = 4

    motor = Motor(motor_a, motor_b, enable=motor_en, pwm=True)
    encoder = RotaryEncoder(enc_a, enc_b, max_steps=0)

    # # Create the speed control
    # sc = MotorSpeedController(0.1, 0.1, motor, encoder)

    # def hook(timestep, enc):
    #     print(f"{timestep=}, {enc=}")
    #     encoder.steps += 1
    # sc.start_thread(0.5, hook)

    # time.sleep(5)
    # print("Setting motor to move")
    # sc.set_target_speed(2)

    # Create the position control
    pc = MotorPositionController(0.01, 0.001, 0.01, 0.001, motor, encoder)
    pc.start_thread(0.5)
    time.sleep(5)
    logging.info("Setting position step")
    pc.set_target_position(100)
