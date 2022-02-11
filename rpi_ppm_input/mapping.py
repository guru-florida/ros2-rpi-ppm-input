
def constrain(min: float, v: float, max: float):
    """ constrain or clamp a number between a min and max value """
    if v < min:
        return min
    elif v > max:
        return max
    else:
        return v


def deadband(v: int, bandwidth: int, center: int = 0):
    """ keep a value at 'center' until a certain threshold is passed. Useful on noisy or uncalibrated joysticks
        where we want to keep the robot at a neutral position until a certain amount of input is given.
    """
    if bandwidth == 0:
        return v
    elif v < center:
        return center if v > center - bandwidth else v + bandwidth
    else:
        return center if v < center + bandwidth else v - bandwidth


def map_throttle(v: int, zero_point: int, min_throttle: int = 0):
    """ map a PPM channel value to a throttle-like output where a certain minimal throttle
        must be seen before activating motion..
    """
    return 0 if v < zero_point else v - zero_point + min_throttle


def map_throttle_reversable(v: int, zero_point: int, with_deadband: int = 0):
    """ map a PPM channel value to a throttle-like output, similar to map_throttle(), but in this case
        the stick is centered and pulling back on mid position indicates a reverse throttle or "go backwards".
    """
    return deadband(v - zero_point, with_deadband)


def map_midstick(v: int, with_deadband: int = 0):
    """ map a PPM channel value to +/- around a midstick position. Useful for roll/pitch/yaw like controls.
    """
    return deadband(v - 1500, with_deadband, 0)


def map_switch(v: int, bands: int):
    """ map a PPM channel to an option switch with the number of bands (toggle position), typical used for
        2 or 3-way switches.
    """
    v = constrain(0, v - 1000, 999)
    bandwidth = 1000 / bands
    return int(v / bandwidth)


def ppm_remap(v: int, min_output: float or int, max_output: float or int, input_offset: int = 1000):
    """ remap the PPM channel range (1000 to 2000) to a scaled floating output value.
        Since other PPM map functions may change the normal 1000 offset to 0 or a +/- value you can
        set the base offset using the 'input_offset' argument.
    """
    if v <= input_offset:
        return min_output
    elif v >= input_offset + 1000:
        return max_output
    elif max_output < min_output:
        # min/max is reversed, so reverse the mapping and call recursively
        # reverse input expression is equivalent to: 1000 - (v - input_offset) + input_offset
        return ppm_remap(2 * input_offset + 1000 - v, max_output, min_output)
    else:
        return min_output + (v - input_offset) / 1000.0 * (max_output - min_output)


