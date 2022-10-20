import numpy as np
import matplotlib.pyplot as pp
import scipy.integrate as integrate
from matplotlib.patches import Rectangle

from math import pi, trunc
from numpy import sin, cos 
#######################################################
def derivatives(state, t):
	ds = np.zeros_like(state)

	_th = state[0]
	_Y = state[1]
	_x = state[2]
	_Z = state[3]

	# x0 = step(t)

	u = Kp_th * _th + Kd_th * _Y + Kp_x * (_x - x0) + Kd_x * _Z

	ds[0] = state[1]
	ds[1] = (g * sin(_th) - u * cos(_th)) / L
	ds[2] = state[3]
	ds[3] = u
	return ds

def trim(x, step):
    d = trunc(x / step)
    return step * d

