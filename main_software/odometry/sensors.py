import time
import busio
import adafruit_tcs34725
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

# Initialize I2C buses
i2c1 = busio.I2C(board.SCL, board.SDA)  # Default I2C bus
i2c3 = busio.I2C(scl=board.D24, sda=board.D23)  # Second I2C bus
# Initialize sensors
sensor_left = adafruit_tcs34725.TCS34725(i2c1)
sensor_right = adafruit_tcs34725.TCS34725(i2c3)

# Function to detect color change
def is_white(rgb, threshold=240):
    """Detect if the color is white by checking if RGB values are close to 255"""
    r, g, b = rgb
    return r > threshold and g > threshold and b > threshold
    # could also do just b < threshold to detect less amount of blue

def main():
  # Main loop to check for color changes
  while True:
      # Get current color values
      rgb_left = sensor_left.color_rgb_bytes
      rgb_right = sensor_right.color_rgb_bytes
      
      # Check for drastic color changes on the left sensor
      if is_white(rgb_left):
          detection_time_left = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
          message_left = f"White color detected on the left at {detection_time_left}"
          logging.info(message_left)
          publish_mqtt(MQTTTopics.ODOMETRY_SENSORS, {"sensor": "left", "event": "white_detected", "time": detection_time_left})
          
      # Check for drastic color changes on the right sensor
      if is_white(rgb_right):
          detection_time_right = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
          message_right = f"White color detected on the right at {detection_time_right}"
          logging.info(message_right)
          publish_mqtt(MQTTTopics.ODOMETRY_SENSORS, {"sensor": "right", "event": "white_detected", "time": detection_time_right})
          
      # Sleep for a while before the next reading
      time.sleep(1.0)


if __name__ == "__main__":
    setup_logging("log_sensors.txt", logging.DEBUG)
    main()