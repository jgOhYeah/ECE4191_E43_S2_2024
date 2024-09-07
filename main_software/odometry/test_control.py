"""test_control.py
Tests the motor controller."""

import gpiozero
from gpiozero import Motor, RotaryEncoder
# from gpiozero.pins.mock import MockFactory, MockPWMPin

# gpiozero.Device.pin_factory = MockFactory(pin_class=MockPWMPin)

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
    motor_a, motor_b, motor_en, enc_a, enc_b = 6, 5, 13, 19, 26

    motor = Motor(motor_a, motor_b, enable=motor_en, pwm=True)
    encoder = RotaryEncoder(enc_a, enc_b, max_steps=0)
    speed_kp = 0.00001
    speed_ki = 0.002
    speed_windup = 0.01
    # Create the speed control
    # sc = MotorSpeedController(0.00001, 0.002, 0.01, motor, encoder)

    # def hook(timestep, enc):
    #     print(f"{timestep=}, {enc=}")
    #     encoder.steps += 1
    # sc.start_thread(0.1)
    # sc.set_target_speed(1000)
    # time.sleep(5)
    # sc.set_target_speed(2000)
    # print("Setting motor to move")
    # sc.set_target_speed(200)
    # motor.forward(1)

    # # Create the position control
    pos_kp = 0.01
    pos_ki = 0.2
    pos_windup = 1
    pc = MotorPositionController(speed_kp, speed_ki, speed_windup, pos_kp, pos_ki, pos_windup, motor, encoder)
    pc.start_thread(0.1)
    logging.info("Setting position step")
    pc.set_target_position(1000, 1000)
    time.sleep(5)

    while True:
        time.sleep(1)