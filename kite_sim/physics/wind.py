"""Power-law wind shear + exactly discretized OU gusts.

Unlike the reference implementation, random-generator state is explicit, reset
is repeatable, and speed is bounded below by zero. Wind direction is expressed
in the FIXED ground-unit frame; varying wind never rotates the physical state.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class WindState:
    gust: float
    rng: tuple[int, int, int, int]


def initial_wind(seed):
    s = np.random.PCG64(seed).state
    return WindState(0.0, (s['state']['state'], s['state']['inc'],
                           s['has_uint32'], s['uinteger']))


def advance_wind(state, p):
    if p.turbulence_intensity == 0:
        return state
    bitgen = np.random.PCG64(0)
    value, inc, has, uint = state.rng
    bitgen.state = {'bit_generator': 'PCG64', 'state': {'state': value, 'inc': inc},
                    'has_uint32': has, 'uinteger': uint}
    draw = np.random.Generator(bitgen).standard_normal()
    a = math.exp(-p.dt/p.turbulence_time_const)
    gust = a*state.gust+p.turbulence_intensity*math.sqrt(1-a*a)*draw
    s = bitgen.state
    return WindState(gust, (s['state']['state'], s['state']['inc'],
                            s['has_uint32'], s['uinteger']))


def wind_velocity(height, state, p):
    speed = max(0.0, p.wind_speed*(max(height, p.wind_z_min)/p.wind_z_ref)
                ** p.shear_exponent*(1+state.gust))
    d = math.radians(p.wind_heading_deg)
    return np.array([speed*math.cos(d), speed*math.sin(d), 0.0])
