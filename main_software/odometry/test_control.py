#!/usr/bin/env python3
"""test_control.py
Tests the motor controller."""

import gpiozero
from gpiozero import Motor, RotaryEncoder

# from gpiozero.pins.mock import MockFactory, MockPWMPin

# gpiozero.Device.pin_factory = MockFactory(pin_class=MockPWMPin)

from motor_control import (
    PIControllerLogged,
    MotorAccelerationController,
    MotorSpeedController,
    MotorPositionController,
)

import time
import logging
import sys

from digitalfilter import create_filter

sys.path.insert(1, sys.path[0] + "/../")
from defines import setup_logging

log_dir = "test"

def test_acceleration(ac:MotorAccelerationController, sc:MotorSpeedController):
    ac.set_target_acceleration(3000)
    prev_time = time.time()
    next_accel = -500
    while True:
        sc.update()
        time.sleep(0.02)

        cur_time = time.time()
        if cur_time > prev_time + 5:
            prev_time = cur_time
            ac.set_target_acceleration(next_accel)
            next_accel = -next_accel

def test_speed(sc:MotorSpeedController):
    sc.start_thread(0.02)
    ts = 1000
    print(f"Setting motor to move at {ts} steps / second.")
    sc.set_target_speed(ts)
    time.sleep(8)
    ts = -2000
    print(f"Setting motor to move at {ts} steps / second.")
    sc.set_target_speed(ts)


# def test_position(speed_kp, speed_ki, speed_windup, pos_kp, pos_ki, pos_windup):
#     pc = MotorPositionController(
#         speed_kp, speed_ki, speed_windup, pos_kp, pos_ki, pos_windup, motor, encoder
#     )
#     pc.start_thread(0.1)
#     logging.info("Setting position step")
#     pc.set_target_position(1000, 3000)
#     time.sleep(5)


if __name__ == "__main__":
    setup_logging()
    logging.info("Control test started")
    # Create the motor and encoder objects.
    motor_a, motor_b, motor_en, enc_a, enc_b = 6, 5, 13, 19, 26
    motor = Motor(motor_a, motor_b, enable=motor_en, pwm=True)
    encoder = RotaryEncoder(enc_a, enc_b, max_steps=0)
    # motor.forward(0.1) # NOTE
    # Create the accelerometer controller
    accel_kp = 0.00001
    accel_ki = 0.001
    accel_windup = 20
    accel_on = 10000
    accel_off = 10000
    accel_control = MotorAccelerationController(
        PIControllerLogged(accel_kp, accel_ki, accel_windup, log_dir, "acceleration.csv"),
        motor,
        encoder,
        accel_on,
        accel_off,
        create_filter(4, 20, 1/0.02)
    )

    # Create the speed controller
    speed_kp = 2
    speed_ki = 0.3
    speed_windup = 3
    speed_control = MotorSpeedController(
        PIControllerLogged(speed_kp, speed_ki, speed_windup, log_dir, "speed.csv"),
        accel_control,
        create_filter(4, 20, 1/0.02)
    )
    # test_acceleration(accel_control, speed_control)

    test_speed(speed_control)
    # pos_kp = 2.5
    # pos_ki = 2
    # pos_windup = 3
    # test_speed(speed_kp, speed_ki, speed_windup)
    # test_position(speed_kp, speed_ki, speed_windup, pos_kp, pos_ki, pos_windup)
    # Create the position control
    

    while True:
        time.sleep(1)
