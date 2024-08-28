"""plot_odometry.py
Tests and plots the path taken to return home."""

import gpiozero
from gpiozero.pins.mock import MockFactory, MockPWMPin
gpiozero.Device.pin_factory = MockFactory(pin_class=MockPWMPin)
from odometry import *
from turtle import Turtle

t = Turtle()

prev_heading = 0

def plot_movement(movement:MovementRecord):
    global prev_heading
    print(f"Plotting move {movement}")
    if movement.type == MovementType.MOVE:
        t.forward(movement.value*200)
    elif movement.type == MovementType.TURN:
        new_heading = prev_heading + 180/math.pi*movement.value
        t.setheading(new_heading)
        prev_heading = new_heading

def plot_history(hist:List[MovementRecord]):
    for movement in hist:
        plot_movement(movement)

# Testing list of moves
moves = [
    MovementRecord(MovementType.TURN, math.pi/2),
    MovementRecord(MovementType.MOVE, 1),
    MovementRecord(MovementType.TURN, math.pi/2),
    MovementRecord(MovementType.TURN, math.pi/2),
    MovementRecord(MovementType.MOVE, 1),
    MovementRecord(MovementType.MOVE, -2),
    MovementRecord(MovementType.TURN, math.pi/2),
    MovementRecord(MovementType.MOVE, 1)
]

# Plot the starting moves
plot_history(moves)

# Calculate how to return home.
print("Calculate heading")
vehicle = Vehicle()
head, dist = vehicle._calculate_origin_move(moves)
return_moves = [
    MovementRecord(MovementType.TURN, head),
    MovementRecord(MovementType.MOVE, dist)
]

# Plot returning home.
t.color(1, 0, 0)
plot_history(return_moves)
t.screen.mainloop()