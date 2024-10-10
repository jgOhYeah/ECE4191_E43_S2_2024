# ball_loader.py
from defines import (
    BallLoadMsg,
    BallLoadFinishMsg,
    setup_logging
)
from threading import Thread
import logging
import time
from gpiozero import DigitalOutputDevice, Button

class BallLoader:
    def __init__(self, motor_pin:int, switch_pin:int):
        self.motor = DigitalOutputDevice(motor_pin)
        self.switch = Button(switch_pin, pull_up=None, active_state=False) # Built in debouncing doesn't work when the motor is on for some reason.
        self.switch.when_pressed = self.turn_off
        # self.activations = 0
        # self.motor.on()
        # time.sleep(1)
        # self.motor.off()
        # self.prev_time = time.time()
        t = Thread(target=self.turn_off)
        t.start()



    def turn_off(self) -> None:
        """If the target number of steps have been reached, turns the motor off.
        """
        # Calculate how long since the last successful press. This must have been at least 0.1s
        # cur_time = time.time()
        # timestep = cur_time - self.prev_time
        # if timestep > 1:
        #     # It has been sufficiently long since the button was last pressed.

        # self.prev_time = cur_time
        # self.activations -= 1
        time.sleep(1)
        #logging.info(f"{self.activations} activations remaining")
        
        #if self.activations <= 0:
        self.motor.off()
        logging.info("Turning off ball loading motor")
        BallLoadFinishMsg().publish()
    
    def turn_on(self, steps:int) -> None:
        """Turns the motor on and waits for a given number of steps to pass.

        Args:
            steps (int): The number of steps.
        """
        self.activations = steps
        self.motor.on()
        logging.info("Turning on ball loading motor")

    def receive_start(self, msg:BallLoadMsg) -> None:
        self.turn_on(msg.ball_count)

    # TODO: Add these features back in.
    # def start_collecting(self):
    #     if self.ball_count < self.capacity:
    #         logging.info("Starting ball collection.")
    #         self.is_collecting = True
    #         # Start the mechanism (e.g., turn on the motor)
    #         self.motor.ChangeDutyCycle(100)  # Adjust as needed
    #         # Implement motor control to collect balls
    #         self.run_motor_collect()
    #     else:
    #         logging.info("Ball loader is at capacity.")
    #         self.notify_capacity_full()

    # def stop_collecting(self):
    #     logging.info("Stopping ball collection.")
    #     self.is_collecting = False
    #     # Stop the mechanism (e.g., turn off the motor)
    #     self.motor.ChangeDutyCycle(0)
    #     # Implement motor stop logic

    # def ball_detected(self):
    #     if self.is_collecting:
    #         self.ball_count += 1
    #         logging.info(f"Ball detected. Total balls: {self.ball_count}")
    #         self.stop_collecting()
    #         self.publish_status()

    #         if self.ball_count >= self.capacity:
    #             self.notify_capacity_full()
    #         else:
    #             # Resume collecting after a short delay
    #             time.sleep(1)  # Adjust delay as needed
    #             self.start_collecting()
    #     else:
    #         logging.warning("Ball detected but not collecting.")

    # def notify_capacity_full(self):
    #     logging.info("Capacity full. Notifying control system.")
    #     publish_mqtt(MQTTTopics.BALL_LOADER_STATUS, {"status": "full"})
    #     # Wait for 'empty' command from control system

    # def dispense_balls(self):
    #     logging.info("Dispensing all balls.")
    #     # Implement dispensing logic (e.g., run motor continuously for one revolution)
    #     self.run_motor_dispense()
    #     self.ball_count = 0
    #     self.publish_status()
    #     # After dispensing, start collecting again
    #     self.start_collecting()

    # def receive_command(self, args: dict):
    #     command = args.get("command", "")
    #     logging.info(f"Received command: {command}")
    #     if command == "empty":
    #         self.dispense_balls()
    #     else:
    #         logging.warning(f"Unknown command: {command}")

    # def to_dict(self) -> dict:
    #     return {
    #         "ball_count": self.ball_count,
    #         "is_collecting": self.is_collecting,
    #     }

    # def publish_status(self):
    #     publish_mqtt(MQTTTopics.BALL_LOADER_STATUS, self.to_dict())

    # def topic_method_pair(self) -> TopicMethodPair:
    #     return TopicMethodPair(self.topic, self.receive_command)

    # # Placeholder methods for motor control
    # def run_motor_collect(self):
    #     # Implement motor control logic to collect balls
    #     logging.info("Motor running to collect balls.")
    #     # For example:
    #     # self.motor.ChangeDutyCycle(collect_speed)
    #     pass

    # def run_motor_dispense(self):
    #     # Implement motor control logic to dispense balls
    #     logging.info("Motor running to dispense balls.")
    #     # For one full revolution
    #     self.motor_steps = 0
    #     while self.motor_steps < self.steps_per_revolution:
    #         # Implement motor stepping logic
    #         # For example:
    #         # GPIO.output(MOTOR_PIN, GPIO.HIGH)
    #         # time.sleep(step_delay)
    #         # GPIO.output(MOTOR_PIN, GPIO.LOW)
    #         # time.sleep(step_delay)
    #         self.motor_steps += 1
    #         # Simulate step delay
    #         time.sleep(0.01)
    #     logging.info("Completed one full revolution for dispensing.")

ball_loader = BallLoader(15, 25)
