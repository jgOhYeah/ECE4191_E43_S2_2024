"""motor_control.py
Classes to aid with motor control."""

# Import the helper functions in defines.py
from typing import Callable
# import sys

# Logging
import logging

logger = logging.getLogger(__name__)

from enum import Enum
from gpiozero import Motor, RotaryEncoder
import threading
import time


class PIController:
    """Class to implement a PI controller in real time."""

    def __init__(
        self, kp: float, ki: float, windup: float, min_output: float, max_output: float
    ):
        """Initialises the controller with a proportional and integral gain.

        Args:
            kp (float): The proportional gain.
            ki (float): The integral gain.
            windup (float): Anti-windup term.
            min_output (float): The minimum output allowed.
            max_output (float): The maximum output allowed.
        """
        self.kp = kp
        self.ki = ki
        self.windup_term = windup
        self.min_output = min_output
        self.max_output = max_output
        self.integral = 0
        self.last_windup = 0

    def update(self, error: float, timestep: float) -> float:
        """Performs one update of the PI controller.

        Args:
            error (float): The error.
            timestep (float): The timestep in seconds between now and the previous reading.

        Returns:
            float: The controller output.
        """
        # Calculate the theoretical output.
        self.integral += (
            self.ki * error + self.windup_term * self.last_windup
        ) * timestep
        recommended = -(self.kp * error + self.integral)

        # Limit to the required bounds and compensate for windup.
        limited = self.limit(recommended)
        self.last_windup = recommended - limited

        # Don
        return limited

    def limit(self, value: float) -> float:
        """Limits the given value to within the bounds set by self.min_output and self.max_output

        Args:
            value (float): The value to limit.

        Returns:
            float: The limited value.
        """
        if value > self.max_output:
            return self.max_output
        elif value < self.min_output:
            return self.min_output
        else:
            return value

    def set_limits(self, min_output: float, max_output: float) -> None:
        """Sets the maximum and minimum output.

        Args:
            min_output (float): The minumum output.
            max_output (float): The maximum output.
        """
        self.min_output = min_output
        self.max_output = max_output


class MotorControlMode(Enum):
    """Enumerator for whether the motor is trying to maintain speed or position."""

    SPEED = 0
    POSITION = 1


class MotorSpeedController:
    """Class and thread for controlling the speed of a motor."""

    def __init__(
        self,
        kp: float,
        ki: float,
        windup: float,
        motor: Motor,
        encoder: RotaryEncoder,
    ):
        """Initialises the motor controller.

        Args:
            kp (float): The proportional argument.
            ki (float): The integral argument.
            windup (float): The anti-windup term.
            motor (Motor): The motor to control.
            encoder (RotaryEncoder): The encoder to control.
        """
        # TODO: Anti-windup.
        # Targets. Need to take a lock before reading or adjusting to avoid race conditions.
        self.target_speed = 0
        self.last_steps = 0
        self.last_time = 0
        self.cur_speed = 0 # Store the current speed in case we are asked for it.

        # Motor and encoder hardware.
        self.motor = motor
        self.encoder = encoder

        # PI Control for speed
        self.speed_control = PIController(kp, ki, windup, -1, 1)

    def set_target_speed(self, speed: float) -> None:
        """Sets the target speed of the motor.

        Args:
            speed (float): The speed to aim for in steps / second.
        """
        logging.debug(f"Setting target speed to {speed} steps / second.")
        self.target_speed = speed

    def update(self, hook: Callable[[float, int], None] = None) -> None:
        """Updates the controller and motor based on a target speed.

        Args:
            timestamp (float, optional): The timestamp that the measurement was taken. Defaults to time.time().
            hook (Callable[[float, int], None], optional): If set, calls this function before every speed control update
                                                           with the timestep and number of encoder steps. Defaults to
                                                           None.
        """
        # Calculate the timestep.
        timestamp = time.time()
        timestep = timestamp - self.last_time
        self.last_time = timestamp

        # Call the hook
        steps = self.encoder.steps
        if hook:
            hook(timestep, steps)

        # Measure the current speed.
        cur_speed = self._measure_speed(timestep, steps)

        # Calculate and set the new motor PWM value.
        error = cur_speed - self.target_speed
        self._set_pwm(self.speed_control.update(error, timestep))

    def _set_pwm(self, pwm: float) -> float:
        """Sets the motor to a given PWM level

        Args:
            pwm (float): The PWM level (>=+1 is flat out forwards, 0 is stopped, <=-1 is flat out backwards).

        Returns:
            float: The difference between the requestated and maximum possible values.
        """
        logging.debug(f"Setting PWM to {pwm}")
        limited = pwm
        if pwm > 0:
            limited = min(pwm, 1)
            self.motor.forward(limited)
        elif pwm < 0:
            limited = max(pwm, -1)
            self.motor.backward(-limited)
        else:
            self.motor.stop()

        return pwm - limited

    def _measure_speed(self, timestep: float, steps: int) -> float:
        """Calculates the current speed in steps per second.

        Args:
            timestep (float): The timestep between now and the previous reading.
            steps (int): The current steps of the encoder.

        Returns:
            float: The current speed in steps per second.
        """
        speed = (steps - self.last_steps) / timestep
        self.last_steps = steps
        self.cur_speed = speed
        return speed

    def run(self, loop_delay: float, hook: Callable[[float, int], None] = None) -> None:
        """Infinite loop to read the motor speed and control it.

        Args:
            loop_delay (float): The delay between control updates.
            hook (Callable[[float, int], None], optional): If set, calls this function before every speed control update
                                                           with the timestep and number of encoder steps. Defaults to
                                                           None.
        """
        logging.debug("Motor controller thread started.")
        while True:
            self.update(hook)
            time.sleep(loop_delay)

    def start_thread(
        self, loop_delay: float, hook: Callable[[float, int], None] = None
    ) -> None:
        """Creates a thread and uses it to run the controller.

        Args:
            loop_delay (float): The delay between control updates.
            hook (Callable[[float, int], None], optional): If set, calls this function before every speed control update
                                                           with the timestep and number of encoder steps. Defaults to
                                                           None.
        """
        thread = threading.Thread(target=self.run, args=(loop_delay, hook))
        thread.start()
    
    def get_speed(self) -> float:
        """Gets the current speed of the motor.

        Returns:
            float: The current speed in steps / second.
        """
        return self.cur_speed


class MotorPositionController:
    """Class for controlling motor position."""

    def __init__(
        self,
        speed_kp: float,
        speed_ki: float,
        speed_windup: float,
        pos_kp: float,
        pos_ki: float,
        pos_windup: float,
        motor: Motor,
        encoder: RotaryEncoder,
    ) -> None:
        """Initialises the controller.

        Args:
            speed_kp (float): Proportional term for the speed controller.
            speed_ki (float): Integral term for the speed controller.
            speed_windup (float): Anti-windup term for the speed controller.
            pos_kp (float): Proportional term for the position controller.
            pos_ki (float): Integral term for the position controller.
            pos_windup (float): Anti-windup term for the position controller.
            motor (Motor): Motor to control.
            encoder (RotaryEncoder): Encoder attached to the motor.
        """
        self.speed_control = MotorSpeedController(
            speed_kp, speed_ki, speed_windup, motor, encoder
        )
        self.position_control = PIController(pos_kp, pos_ki, pos_windup, 0, 0)
        self.target_position = 0
        self.mode = MotorControlMode.POSITION
        self.pos_lock = threading.Lock()

    def set_target_speed(self, speed: float) -> None:
        """Sets the target speed of the motor. Position control will be ignored.

        Args:
            speed (float): The speed to aim for in steps / second.
        """
        self.pos_lock.acquire()
        self.mode = MotorControlMode.SPEED
        self.speed_control.set_target_speed(speed)
        self.pos_lock.release()

    def set_target_position(self, position: int, max_speed: float = None) -> None:
        """Sets the target position to drive to.

        Args:
            position (int): The global position in encoder steps.
            max_speed (float, optional): The maximum speed to traverse at. If None, no limit. Defaults to None.
        """
        # Use locks here as we are editing multiple variables that need to stay in sync / be updated together.
        logging.debug(f"Setting target position to {position} steps.")
        self.pos_lock.acquire()
        self.target_position = position
        self.mode = MotorControlMode.POSITION
        self.position_control.set_limits(-max_speed, max_speed)
        self.pos_lock.release()

    def _position_control_hook(self, timestep: float, steps: int) -> None:
        """Hook that runs the position controller.

        Args:
            timestep (float): The time step from the last time this hook was called.
            steps (int): The current number of steps on the encoder.
        """
        self.pos_lock.acquire()
        # Check we actually need to control the speed.
        if self.mode == MotorControlMode.POSITION:
            # Calculate the speed output from the controller in steps / second.
            print(f"{steps=}, {self.position_control.last_windup=}")
            error = steps - self.target_position
            new_speed = self.position_control.update(error, timestep)

            # Update the target speed.
            self.speed_control.set_target_speed(new_speed)
        self.pos_lock.release()

    def update(self) -> None:
        """Updates the controller.
        """
        self.speed_control.update(self._position_control_hook)

    def run(self, loop_delay: float) -> None:
        """Runs the position controller.

        Args:
            loop_delay (float): The loop delay to use.
        """
        self.speed_control.run(loop_delay, self._position_control_hook)

    def start_thread(self, loop_delay: float) -> None:
        """Starts the control thread.

        Args:
            loop_delay (float): The loop delay to use.
        """
        self.speed_control.start_thread(loop_delay, self._position_control_hook)
    
    def get_speed(self) -> float:
        """Gets the current speed of the motor.

        Returns:
            float: The current speed in steps / second.
        """
        return self.speed_control.cur_speed
