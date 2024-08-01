# Runs the motors forwards and turns.
from gpiozero import Motor
import time

motor1_a, motor1_b = 4, 14
motor2_a, motor2_b = 17, 18
print(f"About to start motors on pins {motor1_a}, {motor1_b}, {motor2_a}, {motor2_b}")
left_motor = Motor(motor1_a, motor1_b)
right_motor = Motor(motor2_a, motor2_b)
while True:
    print("Forwards")
    left_motor.forward()
    right_motor.forward()
    time.sleep(2)
    print("Backwards")
    left_motor.forward()
    right_motor.backward()
    time.sleep(2)