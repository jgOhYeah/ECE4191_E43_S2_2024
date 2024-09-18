"""kalman.py
Classes and functions for implementing a Kalman filter to work out where this is based on where it isn't."""
# import numpy as np
import math
import time

class PositionAccumulator:
    """Class that accumulates position and heading. This isn't a full Kalman filter, but should be ok for short durations.
    """
    def __init__(self, x:float=0, y:float=0, heading:float=0) -> None:
        self.x = x # Starts off as side to side.
        self.y = y # Starts off as forwards and back.
        self.heading = heading
    
    def add_current(self, angular_velocity:float, rotation_centre:float, linear_velocity:float) -> None:
        """Adds the current data for logging purposes if needed.

        This needs to be called once after each call to add_relative if logging is used.
        
        Args:
            angular_velocity (float): _description_
            rotation_centre (float): _description_
            linear_velocity (float): _description_
        """
        pass

    def add_relative(self, forwards_change:float, sideways_change:float, heading_change:float):
        """Adds a relative position change.

        Args:
            forwards_change (float): The forwards change.
            sideways_change (float): The sideways change.
            heading_change (float): The change in heading after the relative change.
        """
        self.y += forwards_change*math.sin(self.heading) + sideways_change*math.cos(self.heading)
        self.x += forwards_change*math.cos(self.heading) + sideways_change*math.sin(self.heading)
        self.heading += heading_change
    
    def __repr__(self) -> str:
        """Generates a string representation.
        """
        return f"Accumulator({self.x}, {self.y}, {self.heading})"

class PositionAccumulatorLogged(PositionAccumulator):
    """Adds logging to the position accumulator."""
    def __init__(self, x:float=0, y:float=0, heading:float=0, log_file:str="position.csv", log_every:int=10):
        super().__init__(x, y, heading)

        self.log_every = log_every
        self.log_count = log_every
        self.enable_current = False
        # Create the log file.
        self.log_file = open(log_file, "w", buffering=1)
        self.log_file.write(f"Timestamp [s],Forwards change [m],Sideways change [m],Heading change [rad],X [m],Y [m],Heading [rad],Angular velocity [rad/s],Rotation centre [m],Linear velocity [m/s]\n")
    
    def add_current(self, angular_velocity: float, rotation_centre: float, linear_velocity: float) -> None:
        if self.enable_current:
            self.log_file.write(f"{angular_velocity},{rotation_centre},{linear_velocity}\n")
            self.enable_current = False

    def add_relative(self, forwards_change: float, sideways_change: float, heading_change: float):
        super().add_relative(forwards_change, sideways_change, heading_change)

        # Log every so often
        self.log_count += 1
        if self.log_count >= self.log_every:
            self.log_count = 0
            self.log_file.write(f"{time.time()},{forwards_change},{sideways_change},{heading_change},{self.x},{self.y},{self.heading},")
            self.enable_current = True
    
    def close(self):
        self.log_file.close()

class Kalman:
    pass