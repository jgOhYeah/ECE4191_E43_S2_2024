"""kalman.py
Classes and functions for implementing a Kalman filter to work out where this is based on where it isn't."""
# import numpy as np
import math
class PositionAccumulator:
    """Class that accumulates position and heading. This isn't a full Kalman filter, but should be ok for short durations.
    """
    def __init__(self, x:float=0, y:float=0, heading:float=0) -> None:
        self.x = x
        self.y = y
        self.heading = heading
    
    def add_relative(self, forwards_change:float, sideways_change:float, heading_change:float):
        """Adds a relative position change.

        Args:
            forwards_change (float): The forwards change.
            sideways_change (float): The sideways change.
            heading_change (float): The change in heading after the relative change.
        """
        self.x += forwards_change*math.sin(self.heading) + sideways_change*math.cos(self.heading)
        self.y += forwards_change*math.cos(self.heading) + sideways_change*math.sin(self.heading)
        self.heading += heading_change
    
    def __repr__(self) -> str:
        """Generates a string representation.
        """
        return f"Accumulator({self.x}, {self.y}, {self.heading})"
class Kalman:
    pass