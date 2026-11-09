"""Ground-frame elevation/azimuth basis for display and initialization."""
import math
import numpy as np


def tangent_frame(theta,phi):
    ct,st,cp,sp = math.cos(theta),math.sin(theta),math.cos(phi),math.sin(phi)
    return np.array([[-cp*st,-sp,-cp*ct],[-sp*st,cp,-sp*ct],[ct,0.,-st]])

