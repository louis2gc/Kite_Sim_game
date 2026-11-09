"""Hamilton scalar-first unit quaternions; active body-to-ground rotation."""
import numpy as np


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array([w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v])


def normalize(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid quaternion')
    return q/norm


def rotation(q):
    w, x, y, z = normalize(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def increment(vector):
    """Quaternion exponential of a rotation vector in radians."""
    vector = np.asarray(vector, dtype=float)
    angle = np.linalg.norm(vector)
    return np.r_[np.cos(angle/2), .5*np.sinc(angle/(2*np.pi))*vector]
