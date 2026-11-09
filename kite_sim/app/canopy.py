"""Illustrative fixed-size canopy; shape is not a simulated flexible wing."""
import numpy as np
from kite_sim.physics.kite import source_position, nose_source
from kite_sim.physics.quaternion import rotation
from kite_sim.physics.rigid_params import geometry


def canopy_vertices(state, params):
    center = source_position(state, params)
    nose = nose_source(state)
    R=rotation(state.quaternion)
    center=center+(R@geometry(params)[0][1])*[1,-1,-1]
    span_axis=R[:,1]*[1,-1,-1]
    radial=-R[:,2]*[1,-1,-1]
    span=params.width*params.relative_width
    # Integral of chord(a)=chord_center*(1-0.65*a*a), a in [-1,1].
    # Planform integrates to the configured area; curvature remains illustrative.
    chord_center = params.area / (span * (1-0.65/3))
    leading, trailing = [], []
    for a in np.linspace(-1, 1, 17):
        bow = -radial * (params.height * a*a)
        lead = (center + span_axis*(a*span/2)
                + nose*chord_center*(0.55-0.97*a*a) + bow)
        tail = lead - nose*(chord_center*(1-0.65*a*a))
        leading.append(lead)
        trailing.append(tail)
    return center, leading, trailing
