# ros2 rpi-ppm-input

This node reads a PPM signal from a Raspberry Pi GPIO pin using the RPi.GPIO library.
PPM signals are often output by RC type radio receivers. I have a Turnigy 9x radio with
a PCM to PPM converter and also a Radiolink R8FM. The R8FM makes a nice simple controller
for robots.

- PCM signals have a wire for each channel and encode the value as a pulse between 1000
  and 2000 microseconds. These mode of signaling is NOT supported by this node.
- PPM signals are similar to PCM, but the 8-12 channels are encoded on one wire where
  each channel is output serially one after the other with a missing pulse indicating
  the start/end of the channel grouping. This single-wire type of RC signaling is what
  this node reads.


# Raspberry PI 3.3v GPIO
*The RPi pins are NOT 5v tolerant!* Most RC transmitters will output 5v TTL level. So be
sure to protect your RPi inputs with a 5v to 3.3v level converter. There are many options
available. 

I used this simple resistor and diode circuit to protect my RPi and it is working but I
make no warranties about it. I got this advice from [tansi.org](https://www.tansi.org/rp/interfacing5v.html)
who got it off the from someone on the RPi community.

[5v to 3.3v pin protection circuit](http://url/to/img.png)


