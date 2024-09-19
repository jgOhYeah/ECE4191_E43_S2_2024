"""motor_control.py
Classes to aid with motor control."""

# Import the helper functions in defines.py
from typing import Callable, Tuple

import os

# Logging
import logging

logger = logging.getLogger(__name__)

from enum import Enum
from gpiozero import Motor, RotaryEncoder
import threading
import time
import math

from digitalfilter import LiveLFilter

class PIController:
    """Class to implement a PI controller in real time."""

    def __init__(
        self, kp: float, ki: float, windup: float
    ):
        """Initialises the controller with a proportional and integral gain.

        Args:
            kp (float): The proportional gain.
            ki (float): The integral gain.
            windup (float): Anti-windup term.
        """
        self.kp = kp
        self.ki = ki
        self.windup_term = windup
        self.min_output = 0
        self.max_output = 0
        self.integral = 0
        self.last_windup = 0
        self.target = 0

    def update(self, current: float, timestep: float) -> float:
        """Performs one update of the PI controller.

        Args:
            current (float): The current reading.
            timestep (float): The timestep in seconds between now and the previous reading.

        Returns:
            float: The controller output.
        """
        # Calculate the error.
        error = self.error(current, self.target)

        # Calculate the theoretical output.
        self.integral += (
            self.ki * error + self.windup_term * self.last_windup
        ) * timestep
        recommended = -(self.kp * error + self.integral)
        # Limit to the required bounds and compensate for windup.
        limited = self.limit(recommended)
        self.last_windup = recommended - limited

        # Done
        return limited

    def error(self, current:float, target:float) -> float:
        """Calculates the error.

        Args:
            current (float): The current measurement.
            target (float): The target.

        Returns:
            float: By default, current - target.
        """
        return current - target

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

    def set_target(self, target:float) -> None:
        """Sets the target.

        Args:
            target (float): The target.
        """
        self.target = target

class PIControllerLogged(PIController):
    def __init__(
        self,
        kp: float,
        ki: float,
        windup: float,
        log_dir: str,
        log_file: str,
    ):
        """Creates the PI controller and opens a file to log to.
        """
        # Initialise as normal.
        super().__init__(kp, ki, windup)

        # Create the log directory if needed
        if not os.path.exists(log_dir):
            logging.info(f"Creating folder '{log_dir}'")
            os.makedirs(log_dir)

        # Create the log file.
        self.log_file = open(f"{log_dir}/{log_file}", "w", buffering=1)
        self.log_file.write(f"Timestamp [s],Timestep [s],Limit Min,Limit Max,Target,Windup,Last Windup,Current,Output\n")
    
    def update(self, current: float, timestep: float) -> float:
        output = super().update(current, timestep)
        self.log_file.write(f"{time.time()},{timestep},{self.min_output},{self.max_output},{self.target},{self.windup_term},{self.last_windup},{current},{output}\n")
        return output

    def close(self):
        self.log_file.close()

class PIControllerAngle(PIControllerLogged):
    """Limits the maximum angles in error calculations to +- pi
    """

    def error(self, current: float, target: float) -> float:
        error = current - target
        error = error % (2*math.pi)
        if error > math.pi:
            error -= 2*math.pi
        
        return error

class MotorControlMode(Enum):
    """Enumerator for whether the motor is trying to maintain speed or position."""

    SPEED = 0
    POSITION = 1


class MotorAccelerationController:
    """Class for controlling motor acceleration."""

    def __init__(
        self,
        controller: PIController,
        motor: Motor,
        encoder: RotaryEncoder,
        max_accel_on: float,
        max_accel_off: float,
        filter: LiveLFilter
    ) -> None:
        """Initialised the controller.

        Args:
            controller (PIController): The controller to manage the acceleration.
            motor (Motor): The motor.
            encoder (RotaryEncoder): The encoder.
            max_accel_on (float): Maximum acceleration magnitude when speeding up in steps / s^2.
            max_accel_off (float): Maximum acceleration magnitude when slowing down in steps / s^2.
            filter (LiveLFilter): Filter to use on the acceleration data.
        """
        self.acceleration_control = controller
        self.acceleration_control.set_limits(-1, 1)
        self.motor = motor
        self.encoder = encoder
        self.max_accel_on = max_accel_on
        self.max_accel_off = max_accel_off
        self.last_speed = 0
        self.filter = filter

    def set_target_acceleration(self, acceleration: float) -> None:
        """Sets the target acceleration.

        Args:
            acceleration (float): Target in steps / second ^2.
        """
        self.acceleration_control.set_target(acceleration)

    def _measure_acceleration(self, timestep: float, speed: float) -> float:
        """Measures the current acceleration.

        Args:
            timestep (float): The time step.
            speed (float): The latest speed reading in steps / second.

        Returns:
            float: The acceleration in steps / second ^2
        """
        # Calculate instantaneous acceleration.
        cur_acceleration = (speed - self.last_speed) / timestep
        self.last_speed = speed

        # Low pass filter
        ave_acceleration = self.filter(cur_acceleration)

        # print(f"{timestep=}, {speed=}, {cur_acceleration=}, {ave_acceleration=}")
        return ave_acceleration

    def _set_pwm(self, pwm: float) -> float:
        """Sets the motor to a given PWM level

        Args:
            pwm (float): The PWM level (>=+1 is flat out forwards, 0 is stopped, <=-1 is flat out backwards).

        Returns:
            float: The difference between the requestated and maximum possible values.
        """
        # logging.debug(f"Setting PWM to {pwm}")
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

    def update(self, timestep: float, speed: float) -> None:
        """Updates the controller and motor based on a target acceleration.

        Args:
            timestep (float): The time since the last update.
            speed (float): The current speed.
        """
        # Calculate the acceleration and error.
        cur_acceleration = self._measure_acceleration(timestep, speed)
        
        # Set PWM limits
        min_pwm, max_pwm = self.get_pwm_limits(speed)
        # min_pwm, max_pwm = -1, 1
        self.acceleration_control.set_limits(min_pwm, max_pwm)

        # Use the controller to calculate a new value
        pwm_request = self.acceleration_control.update(cur_acceleration, timestep)
        self._set_pwm(pwm_request)

    def get_pwm_limits(self, speed: float) -> Tuple[float, float]:
        """Calculates limits for PWM in the current direction (stops rapidly oscillating between full forwards and
        backwards, needs to go through 0 speed first).

        (Disabling active motor braking in effect).

        Args:
            speed (float): The current speed.

        Returns:
            Tuple[float, float]: The minimum and maximum PWM values respectively.
        """
        if speed > 0:
            return 0, 1
        elif speed < 0:
            return -1, 0
        else:
            # Can go either way
            return -1, 1

    def get_accel_limit(self, speed: float) -> Tuple[float, float]:
        """Returns the maximum and minimum limits to the acceleration, based on the direction of the motors.

        Args:
            speed (float): The current speed.

        Returns:
            Tuple[float, float]: Acceleration limits (minimum, maximum).
        """
        # Calculate the minimum and maximum acceleration based on the current direction.
        if speed > 0:
            # Currently moving forwards.
            max_accel = self.max_accel_on
            min_accel = -self.max_accel_off
        elif speed < 0:
            # Currently moving backwards.
            max_accel = self.max_accel_off
            min_accel = -self.max_accel_on
        else:
            # Not moving, limit in both directions
            max_accel = self.max_accel_on
            min_accel = -self.max_accel_on

        return min_accel, max_accel


class MotorSpeedController:
    """Class and thread for controlling the speed of a motor."""

    def __init__(
        self,
        controller: PIController,
        accel_control: MotorAccelerationController,
        filter: LiveLFilter
    ):
        """Initialises the motor controller.

        Args:
            controller (PIController): PI Controller to use for speed control.
            accel_control (MotorAccelerationController): Controller for motor acceleration.
            filter (LiveLFilter): Filter to use on the speed data.
        """
        # Targets. Need to take a lock before reading or adjusting to avoid race conditions.
        self.last_steps = 0
        self.last_time = time.time()
        self.cur_speed = 0  # Store the current speed in case we are asked for it.

        self.accel_control = accel_control
        self.filter = filter

        # PI Control for speed
        self.speed_control = controller

        self.update_count = 0

    def set_target_speed(self, speed: float) -> None:
        """Sets the target speed of the motor.

        Args:
            speed (float): The speed to aim for in steps / second.
        """
        self.speed_control.set_target(speed)

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
        steps = self.accel_control.encoder.steps
        if hook:
            hook(timestep, steps)

        # Measure the current speed.
        cur_speed = self._measure_speed(timestep, steps)

        self.update_count += 1
        if self.update_count == 6:
            self.update_count = 0
            # Limit the maximum acceleration based on the current direction
            min_accel, max_accel = self.accel_control.get_accel_limit(cur_speed)
            self.speed_control.set_limits(min_accel, max_accel)

            # Update the speed and acceleration controllers.
            accel_request = self.speed_control.update(cur_speed, timestep)
            self.accel_control.set_target_acceleration(accel_request)
        self.accel_control.update(timestep, cur_speed)

    def _measure_speed(self, timestep: float, steps: int) -> float:
        """Calculates the current speed in steps per second.

        Args:
            timestep (float): The timestep between now and the previous reading.
            steps (int): The current steps of the encoder.

        Returns:
            float: The current speed in steps per second.
        """
        speed = (steps - self.last_steps) / timestep
        speed = self.filter(speed)
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
        controller: PIController,
        speed_control: MotorSpeedController,
    ) -> None:
        """Initialises the controller.

        Args:
            controller (PIController): Controller to use to control the position.
            speed_control (MotorSpeedController): Speed controller.
        """
        self.speed_control = speed_control
        self.position_control = controller
        self.mode = MotorControlMode.POSITION
        self.pos_lock = threading.Lock()

        self.update_count = 0

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
        self.pos_lock.acquire()
        self.position_control.set_target(position)
        self.mode = MotorControlMode.POSITION
        self.position_control.set_limits(-max_speed, max_speed)
        self.pos_lock.release()

    def _position_control_hook(self, timestep: float, steps: int) -> None:
        """Hook that runs the position controller.

        Args:
            timestep (float): The time step from the last time this hook was called.
            steps (int): The current number of steps on the encoder.
        """
        self.update_count += 1
        if self.update_count == 20:
            self.update_count = 0
            self.pos_lock.acquire()
            # Check we actually need to control the speed.
            if self.mode == MotorControlMode.POSITION:
                # Calculate the speed output from the controller in steps / second.
                new_speed = self.position_control.update(steps, timestep)

                # Update the target speed.
                self.speed_control.set_target_speed(new_speed)
            self.pos_lock.release()

    def update(self) -> None:
        """Updates the controller."""
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
        return self.speed_control.get_speed()

    def pos_movement_complete(
        self, allowed_error: int = 50, allowed_speed: int = 50
    ) -> bool:
        """Checks if the movement is complete.

        Returns:
            bool: True if moved to new position and stopped, false otherwise.
        """
        self.pos_lock.acquire()
        error = abs(self.speed_control.accel_control.encoder.steps - self.position_control.target)
        speed = self.speed_control.speed_control.target
        self.pos_lock.release()
        return error <= allowed_error and speed <= allowed_speed
