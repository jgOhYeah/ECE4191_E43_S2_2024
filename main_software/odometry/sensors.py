import time
import busio
import adafruit_tcs34725
from adafruit_extended_bus import ExtendedI2C as I2C
import board
from datetime import datetime 

import sys
sys.path.insert(1, sys.path[0] + "/../")

from defines import (
    setup_logging,
    MQTTTopics,
    publish_mqtt
)

import logging
logger = logging.getLogger(__name__)

# Initialize the first I2C bus (i2c-1)
i2c1 = I2C(1)  # Use bus 1
# Initialize the second I2C bus (i2c-3)
i2c3 = I2C(3)  # Use bus 3

sensor_left = None
sensor_right = None

# Initialize the sensors on the respective buses
def initialised_sensors() -> bool:
    """Initialises the colour sensors.

    Returns:
        bool: True if both were initialised successfully, False otherwise.
    """
    global sensor_left
    global sensor_right
    try:
        sensor_left = adafruit_tcs34725.TCS34725(i2c1)
        sensor_right = adafruit_tcs34725.TCS34725(i2c3)
    except ValueError:
        logging.error("Could not initialise 1 or more colour sensors")
        return False
    else:
        # Set integration time and gain for both sensors
        sensor_left.integration_time = 200
        sensor_left.gain = 60
        sensor_right.integration_time = 200
        sensor_right.gain = 60
        return True

# Function to detect color change
def is_white(rgb, threshold=200):
    """Detect if the color is white by checking if RGB values are close to 255"""
    r, g, b = rgb
    return r > threshold and g > threshold
    # could also do just b < threshold to detect less amount of blue

def main():
  # Main loop to check for color changes
  while True:
      # Get current color values
      rgb_left = sensor_left.color_rgb_bytes
      print("rgb_left", rgb_left)
      rgb_right = sensor_right.color_rgb_bytes
      print("rgb_right", rgb_right)
      
      # Check for drastic color changes on the left sensor
      if is_white(rgb_left):
          detection_time_left = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
          message_left = f"White color detected on the left at {detection_time_left}"
          print(message_left)
          logging.info(message_left)
          publish_mqtt(MQTTTopics.ODOMETRY_SENSORS, {"sensor": "left", "event": "white_detected", "time": detection_time_left})
          
      # Check for drastic color changes on the right sensor
      if is_white(rgb_right):
          detection_time_right = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
          message_right = f"White color detected on the right at {detection_time_right}"
          print(message_right)
          logging.info(message_right)
          publish_mqtt(MQTTTopics.ODOMETRY_SENSORS, {"sensor": "right", "event": "white_detected", "time": detection_time_right})
          
      # Sleep for a while before the next reading
      time.sleep(1.0)


if __name__ == "__main__":
    setup_logging("log_sensors.txt", logging.DEBUG)
    if initialised_sensors():
        main()
    else:
        logging.info("Not running the main loop as I can't talk to both colour sensors")