#!/usr/bin/env python

import time
import os
import typing
import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Joy

from .mapping import constrain, deadband, map_throttle, map_throttle_reversable, map_midstick, map_switch, ppm_remap

# default input wired to the PPM receiver
DEFAULT_IN_PPM=7

# debug using these scenarios:
# !python3 ./__init__.py --ros-args -p mode:="array"
# !python3 ./__init__.py --ros-args -p mode:="joy"


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


class Axis:
    GENERIC = 0
    MIDSTICK = 1
    THROTTLE = 2
    REVERSABLE_THROTTLE = 3
    TRIM = 4

    type: int = 0
    deadband: int = 0       # amount of deadband
    _zero_point: int = 1200   # start of throttle (less than this point is 0)
    min_throttle: int = 0   # minimal amount of throttle; once zero_point is reached, this becomes the starting throttle value
    center: int = 0.0
    scale: float = 1.0

    def set_type(self, at: str or int):
        if isinstance(at, str):
            if at == 'generic':
                at = Axis.GENERIC
            elif at == 'midstick':
                at = Axis.MIDSTICK
            elif at == 'throttle':
                at = Axis.THROTTLE
            elif at == 'reversable-throttle':
                at = Axis.REVERSABLE_THROTTLE
            elif at == 'trim':
                at = Axis.TRIM
            else:
                raise ValueError(f"invalid Axis type '{at}' specified")
        self.type = at
        if self.type == Axis.MIDSTICK:
            self.center = 1500
        elif self.type == Axis.THROTTLE:
            self.center = 1000
            self.zero_point = 1200
        elif self.type == Axis.REVERSABLE_THROTTLE:
            self.center = 1500
            self.deadband = 25
        elif self.type == Axis.TRIM:
            self.center = 1000
            self.zero_point = 1000

    @property
    def zero_point(self):
        return self._zero_point

    @zero_point.setter
    def zero_point(self, v: int):
        self._zero_point = constrain(1000, v, 2000)

    def map(self, v: int):
        # type logic should output between +/- 1000
        if self.type == Axis.MIDSTICK:
            v = map_midstick(v, self.deadband) * 2
        elif self.type == Axis.THROTTLE:
            v = map_throttle(v, self.zero_point, self.min_throttle) * 1000.0 / (2000.0 - self._zero_point)
        elif self.type == Axis.REVERSABLE_THROTTLE:
            v = map_throttle_reversable(v, self.zero_point, self.deadband)
        elif self.type == Axis.TRIM:
            v = constrain(0, v - 1000, 1000)
        else: # assume GENERIC
            # same as MIDSTICK
            v = map_midstick(v, self.deadband) * 2

        return (v/1000.0) * self.scale


class Button:
    poles: int       # how many throw positions

    def __init__(self):
        self.poles = 2

    def map(self, v: int):
        return map_switch(v, self.poles)


class PPMNode(Node):
    mode: str
    frame_id: str
    pin: int = DEFAULT_IN_PPM

    channels: typing.List[Axis or Button]
    highest_channel: int = 0

    axis: typing.List[int]
    buttons: typing.List[int]
    

    def __init__(self):
        super().__init__("ppm_input")

        self.declare_parameters(
                namespace='',
                parameters=[
                    ('mode', 'array'),
                    ('frame_id', ''),
                    ('pin', DEFAULT_IN_PPM)
                    ])
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('joystick.axis', [0,1,2,3,7]),
                    ('joystick.buttons', [4, 6, 5]),
                    ('joystick.3-way', [4, 6])
                    ])

        self.mode = str(self.get_parameter('mode').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.pin = int(self.get_parameter('pin').value)

        # by default we have no channels assigned
        self.channels = [None]*16

        axis_parameter_set = parameters=[
                ('type', Parameter.Type.STRING),
                ('deadband', Parameter.Type.INTEGER),
                ('zero-point', Parameter.Type.INTEGER),
                ('min-throttle', Parameter.Type.INTEGER),
                ('center', Parameter.Type.INTEGER),
                ('scale', Parameter.Type.INTEGER)]

        self.axis = self.get_parameter('joystick.axis').value
        for i, ax in enumerate(self.axis):
            if ax > len(self.channels):
                self.get_logger().error(f"channel {ax} out of range; maximum of 16 channels supported, check your axis parameter")
                exit(-1)
            else:
                ch = Axis()
                self.channels[ax] = ch
                self.highest_channel = max(self.highest_channel, ax)

                # load more advanced axis options (if any exist)
                axis_name = f'axis-{i}'
                # todo: must prefix params here due to bug (https://github.com/ros2/rclpy/issues/894)
                new_params = [(f'{axis_name}.{p}', dv) for p,dv in axis_parameter_set]
                self.declare_parameters(
                        namespace='',
                        parameters=new_params)
                for p, def_v in axis_parameter_set:
                    p_fullname = f'{axis_name}.{p}'
                    param = self.get_parameter_or(p_fullname, None)
                    if param.type_ != Parameter.Type.NOT_SET:
                        if p == 'type':
                            ch.set_type(param.value)
                            self.get_logger().info(f"{axis_name} (ch{ax}) set axis type to {param.value}")
                        else:
                            setattr(ch, p_fullname.replace('-', '_'), param.value)
                            self.get_logger().info(f"{axis_name} (ch{ax}) set {p} to {param.value}")

        self.buttons = self.get_parameter('joystick.buttons').value
        for b in self.buttons:
            if b > len(self.channels):
                self.get_logger().error(f"channel {ax} out of range; maximum of 16 channels supported, check your buttons parameter")
                exit(-1)
            else:
                self.channels[b] = Button()
                self.highest_channel = max(self.highest_channel, b)

        way3 = self.get_parameter('joystick.3-way').value
        for b in way3:
            if b > len(self.channels):
                self.get_logger().error(f"channel {ax} out of range; maximum of 16 channels supported, check your 3-way parameter")
                exit(-1)
            elif not isinstance(self.channels[b], Button):
                self.get_logger().error(f"channel {ax} is not a button; check your 3-way parameter")
                exit(-1)
            else:
                self.channels[b].poles = 3


        # create the publisher depending on our mode
        if self.mode == 'joy':
            self.ppm_publisher_ = self.create_publisher(
                Joy, "input/ppm", 10)
            self.temperature_timer_ = self.create_timer(
                0.1, self.publish_ppm_joy)
        elif self.mode == 'array':
            self.ppm_publisher_ = self.create_publisher(
                Int16MultiArray, "input/ppm", 10)
            self.temperature_timer_ = self.create_timer(
                0.1, self.publish_ppm_array)
        else:
            self.get_logger().error(f"mode '{self.mode}' not supported; expected joy or array")
            exit(-1)


    def publish_ppm_array(self):
        msg = Int16MultiArray()
        msg.data = channels
        self.ppm_publisher_.publish(msg)

    def publish_ppm_joy(self):
        if len(channels) < self.highest_channel:
            self.get_logger().warn(f'channel framing error, got {len(channels)} channels, require {self.highest_channel}')
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for ax in self.axis:
            ch = self.channels[ax]
            v = channels[ax]
            msg.axes.append(ch.map(v))
        for b in self.buttons:
            ch = self.channels[b]
            v = channels[b]
            msg.buttons.append(ch.map(v))
        self.ppm_publisher_.publish(msg)

    def setup_gpio(self):
        # set GPIO pins to be compatible with the RPi header (all boards)
        GPIO.setmode(GPIO.BOARD)

        # configure the PPM input
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # setup a callback 
        GPIO.add_event_detect(self.pin, GPIO.RISING)
        GPIO.add_event_callback(self.pin, cbf)


def main(args=None):
    rclpy.init(args=args)

    node = PPMNode()
    node.setup_gpio()
    rclpy.spin(node)
    rclpy.shutdown()

    GPIO.cleanup()

if __name__ == "__main__":
    main()


