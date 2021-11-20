#!/usr/bin/env python

import time
import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

IN_PPM=7


start_of_frame = False
channels = []
current_frame = []
last_tick = None

# portions of this code was adapted from this SO article but
# originally used pigpio which I then ported to RPi.GPIO
# https://raspberrypi.stackexchange.com/questions/104408/how-to-use-rpi-python-to-read-write-convert-to-from-arduino-ppm-and-pwm-servo-c
def cbf(chpin):
   tick = time.time_ns()
   global start_of_frame, channels, current_frame, last_tick
   if last_tick is not None:
      diff = (tick - last_tick) / 1000
      if diff > 3000: # start of frame
         #line = [f'{v:5.0f}' for v in channels]
         #print(' '.join(line))
         channels = current_frame
         current_frame = []
         start_of_frame = True
      else:
         if start_of_frame:
            if len(channels) < 12:
               current_frame.append(int(diff))
   last_tick = tick

class PPMNode(Node):
    def __init__(self):
        super().__init__("ppm_input")
        self.ppm_publisher_ = self.create_publisher(
            Int16MultiArray, "input/ppm", 10)
        self.temperature_timer_ = self.create_timer(
            0.1, self.publish_ppm)

    def publish_ppm(self):
        msg = Int16MultiArray()
        msg.data = channels
        self.ppm_publisher_.publish(msg)

    def setup_gpio(self):
        # set GPIO pins to be compatible with the RPi header (all boards)
        GPIO.setmode(GPIO.BOARD)

        # configure the PPM input
        GPIO.setup(IN_PPM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # setup a callback 
        GPIO.add_event_detect(IN_PPM, GPIO.RISING)
        GPIO.add_event_callback(IN_PPM, cbf)


def main(args=None):
    rclpy.init(args=args)

    node = PPMNode()
    node.setup_gpio()
    rclpy.spin(node)
    rclpy.shutdown()

    GPIO.cleanup()

if __name__ == "__main__":
    main()


