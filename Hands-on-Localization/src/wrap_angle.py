#!/usr/bin/python

import numpy as np
import math

def wrap_angle(ang):
    if isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
        return ang
    else:
        return ang + (2.0 * math.pi * math.floor((math.pi - ang) / (2.0 * math.pi)))
 
x = 3.15
y = wrap_angle(x)
print(y)

