# ball_loader.py

from defines import (
    MQTTTopics,
    MQTTTopicImplementation,
    publish_mqtt,
    TopicMethodPair,
)
import logging
import time
import RPi.GPIO as GPIO  

class BallLoader(MQTTTopicImplementation):
    def __init__(self, callback=None):
        super().__init__(MQTTTopics.BALL_LOADER_COMMAND, callback)
        self.ball_count = 0          # Number of balls currently in the mechanism
        self.capacity = 3            # Maximum capacity of the mechanism
        self.is_collecting = False   # Whether the mechanism is currently collecting
        self.motor_steps = 0         # Steps taken by the motor (for revolution tracking)
        self.steps_per_revolution = 200  # Number of steps for a full revolution (example value)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_PIN, GPIO.OUT)
        self.motor = GPIO.PWM(MOTOR_PIN, 100)
        self.motor.start(0)

        # Start collecting immediately
        self.start_collecting()

    def start_collecting(self):
        if self.ball_count < self.capacity:
            logging.info("Starting ball collection.")
            self.is_collecting = True
            # Start the mechanism (e.g., turn on the motor)
            self.motor.ChangeDutyCycle(100)  # Adjust as needed
            # Implement motor control to collect balls
            self.run_motor_collect()
        else:
            logging.info("Ball loader is at capacity.")
            self.notify_capacity_full()

    def stop_collecting(self):
        logging.info("Stopping ball collection.")
        self.is_collecting = False
        # Stop the mechanism (e.g., turn off the motor)
        self.motor.ChangeDutyCycle(0)
        # Implement motor stop logic

    def ball_detected(self):
        if self.is_collecting:
            self.ball_count += 1
            logging.info(f"Ball detected. Total balls: {self.ball_count}")
            self.stop_collecting()
            self.publish_status()

            if self.ball_count >= self.capacity:
                self.notify_capacity_full()
            else:
                # Resume collecting after a short delay
                time.sleep(1)  # Adjust delay as needed
                self.start_collecting()
        else:
            logging.warning("Ball detected but not collecting.")

    def notify_capacity_full(self):
        logging.info("Capacity full. Notifying control system.")
        publish_mqtt(MQTTTopics.BALL_LOADER_STATUS, {"status": "full"})
        # Wait for 'empty' command from control system

    def dispense_balls(self):
        logging.info("Dispensing all balls.")
        # Implement dispensing logic (e.g., run motor continuously for one revolution)
        self.run_motor_dispense()
        self.ball_count = 0
        self.publish_status()
        # After dispensing, start collecting again
        self.start_collecting()

    def receive_command(self, args: dict):
        command = args.get("command", "")
        logging.info(f"Received command: {command}")
        if command == "empty":
            self.dispense_balls()
        else:
            logging.warning(f"Unknown command: {command}")

    def to_dict(self) -> dict:
        return {
            "ball_count": self.ball_count,
            "is_collecting": self.is_collecting,
        }

    def publish_status(self):
        publish_mqtt(MQTTTopics.BALL_LOADER_STATUS, self.to_dict())

    def topic_method_pair(self) -> TopicMethodPair:
        return TopicMethodPair(self.topic, self.receive_command)

    # Placeholder methods for motor control
    def run_motor_collect(self):
        # Implement motor control logic to collect balls
        logging.info("Motor running to collect balls.")
        # For example:
        # self.motor.ChangeDutyCycle(collect_speed)
        pass

    def run_motor_dispense(self):
        # Implement motor control logic to dispense balls
        logging.info("Motor running to dispense balls.")
        # For one full revolution
        self.motor_steps = 0
        while self.motor_steps < self.steps_per_revolution:
            # Implement motor stepping logic
            # For example:
            # GPIO.output(MOTOR_PIN, GPIO.HIGH)
            # time.sleep(step_delay)
            # GPIO.output(MOTOR_PIN, GPIO.LOW)
            # time.sleep(step_delay)
            self.motor_steps += 1
            # Simulate step delay
            time.sleep(0.01)
        logging.info("Completed one full revolution for dispensing.")
