#!/usr/bin/env python3
# Runs the motors forwards and turns.
from gpiozero import Motor, LED
import time


left_a, left_b, left_en = 7, 16, 12
right_a, right_b, right_en = 5, 6, 13

# Set enable pins high.
left_enable = LED(left_en)
right_enable = LED(right_en)
left_enable.on()
right_enable .on()

print(f"About to start motors on pins {left_a}, {left_b}, {right_a}, {right_b}")
left_motor = Motor(left_a, left_b)
right_motor = Motor(right_a, right_b)
while True:
    print("Forwards")
    left_motor.forward()
    right_motor.forward()
    time.sleep(2)
    print("Spin 1")
    left_motor.forward()
    right_motor.backward()
    time.sleep(2)
    print("Backwards")
    left_motor.backward()
    right_motor.backward()
    time.sleep(2)
    print("Spin 2")
    left_motor.backward()
    right_motor.forward()
    time.sleep(2)