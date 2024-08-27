#!/usr/bin/env python3
from gpiozero import Motor, LED
from gpiozero import Button
import time
import math

# Motor and Encoder setup (as before)
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

encoder_left_a = 20
encoder_left_b = 21
encoder_right_a = 19
encoder_right_b = 26

left_encoder = Button(encoder_left_a)
right_encoder = Button(encoder_right_a)

left_count = 0
right_count = 0

def left_encoder_tick():
    global left_count
    left_count += 1
    print(f"Left Encoder Tick: {left_count}")

def right_encoder_tick():
    global right_count
    right_count += 1
    print(f"Right Encoder Tick: {right_count}")

left_encoder.when_pressed = left_encoder_tick
right_encoder.when_pressed = right_encoder_tick

wheel_radius = 0.027  # in meters
wheel_base = 0.20  # distance between wheels in meters
ticks_per_revolution = 48

x, y, theta = 0.0, 0.0, 0.0

def update_odometry():
    print("updating odometry")
    global x, y, theta, left_count, right_count
    left_distance = (2 * math.pi * wheel_radius * left_count) / ticks_per_revolution
    right_distance = (2 * math.pi * wheel_radius * right_count) / ticks_per_revolution
    left_count = 0
    right_count = 0
    distance = (left_distance + right_distance) / 2.0
    delta_theta = (right_distance - left_distance) / wheel_base
    x += distance * math.cos(theta + delta_theta / 2.0)
    y += distance * math.sin(theta + delta_theta / 2.0)
    theta += delta_theta
    print(f"Position: x={x:.2f}, y={y:.2f}, theta={theta:.2f} radians")

def rotate(angle):
    print("rotating")
    # Rotate robot by a specific angle (radians)
    target_angle = theta + angle
    while abs(theta - target_angle) > 0.01:
        if angle > 0:
            left_motor.forward(0.5)
            right_motor.backward(0.5)
        else:
            left_motor.backward(0.5)
            right_motor.forward(0.5)


        # Check encoder states and counts
        left_state = left_encoder.is_pressed
        right_state = right_encoder.is_pressed
        print(f"Left Encoder A State: {left_state}, Right Encoder A State: {right_state}")
        print(f"Left Encoder Tick Count: {left_count}, Right Encoder Tick Count: {right_count}")
        
        
        update_odometry()
        time.sleep(0.01)
    left_motor.stop()
    right_motor.stop()

def move_forward(distance):
    print("moving forwards")
    # Move robot forward by a specific distance
    start_x, start_y = x, y
    while math.sqrt((x - start_x)**2 + (y - start_y)**2) < distance:
        left_motor.forward(0.5)
        right_motor.forward(0.5)

        # Check encoder states and counts
        left_state = left_encoder.is_pressed
        right_state = right_encoder.is_pressed
        print(f"Left Encoder A State: {left_state}, Right Encoder A State: {right_state}")
        print(f"Left Encoder Tick Count: {left_count}, Right Encoder Tick Count: {right_count}")
        

        update_odometry()
        time.sleep(0.01)
    left_motor.stop()
    right_motor.stop()

def return_to_origin():
    print("returning to origin")
    # Calculate distance and angle to origin
    distance_to_origin = math.sqrt(x**2 + y**2)
    angle_to_origin = math.atan2(y, x) - theta
    
    # Rotate to face the origin
    print(f"Rotating to face origin by {angle_to_origin:.2f} radians")
    rotate(angle_to_origin)
    
    # Move forward to the origin
    print(f"Moving forward by {distance_to_origin:.2f} meters to the origin")
    move_forward(distance_to_origin)
    
    # Correct final orientation (if needed)
    rotate(-theta)  # Rotate to face the initial orientation (theta = 0)
    print("Returned to origin")

# Example usage:
# Step 1: Move forward 10 cm (0.1 meters)
print("Moving forward 20 cm")
move_forward(0.2)

# Step 2: Turn 90 degrees (π/2 radians)
print("Turning 90 degrees")
rotate(math.pi / 2)

# Step 3: Move forward another 10 cm
print("Moving forward another 10 cm")
move_forward(0.1)

# Step 4: Return to origin
print("Returning to origin")
return_to_origin()
