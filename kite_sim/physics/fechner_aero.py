"""Fechner et al. (2015), Eqs. 47–59, adapted to a rigid body.

https://arxiv.org/abs/1406.6218 (v3). Polar knots are a snapshot of
https://github.com/OpenSourceAWE/KiteUtils.jl/blob/main/data/settings.yaml
(accessed 2026-11-08), not measurements of a specific kite. Linear interpolation
replaces the reference spline. No flexible canopy or dynamic stall model.
"""
import numpy as np
from .quaternion import rotation
from .rigid_params import geometry
from .wind import wind_velocity


def polar(alpha_deg, p):
    angle = (np.asarray(alpha_deg)+180)%360-180
    return np.interp(angle, p.alpha_cl, p.cl), np.interp(angle, p.alpha_cd, p.cd)


def loads(position, velocity, q, omega, delta_m, depower, wind, p):
    R = rotation(q)
    sites = geometry(p)[0][1:]  # B top, C and D sides; no force at nose A.
    world = np.asarray(position)+sites@R.T
    W = np.array([wind_velocity(point[2], wind, p) for point in world])
    apparent = (W-np.asarray(velocity))@R - np.cross(omega, sites)
    projected = apparent.copy()
    projected[0,1] = 0.
    projected[1:,2] = 0.
    speed = np.linalg.norm(apparent, axis=1)
    projected_speed = np.linalg.norm(projected, axis=1)
    # Published unsigned incidence convention, Eqs. 49–51. At zero
    # projected speed angle is immaterial: corresponding lift is zero.
    alpha = np.degrees(np.pi-np.arccos(np.clip(
        projected[:,0]/np.maximum(projected_speed, p.aero_epsilon), -1, 1)))
    # Our positive motor travel shortens the C-side line and turns LEFT.
    # Fechner's normalized side-incidence input uses the opposite sign.
    # This models differential aerodynamic loading, not an added yaw torque.
    twist = -p.steering_twist_deg*(delta_m/p.delta_m_max)/(1+p.k_ds*depower)
    alpha += [p.alpha_zero_deg-p.depower_angle_deg*depower,
              p.alpha_tip_deg-twist, p.alpha_tip_deg+twist]
    cl, cd = polar(alpha, p)
    axes = np.array([[0,1,0], [0,0,1], [0,0,-1]])
    lift_direction = np.cross(apparent, axes)
    norm = np.linalg.norm(lift_direction, axis=1)
    lift_direction /= np.maximum(norm[:,None], p.aero_epsilon)
    areas = p.area*np.array([1.,p.side_area_ratio,p.side_area_ratio])
    lift = (.5*p.rho*areas*projected_speed**2*cl)[:,None]*lift_direction
    drag = (.5*p.rho*areas*.93*(1-p.side_area_ratio)*speed*cd)[:,None]*apparent
    forces = lift+drag
    return R@forces.sum(axis=0), np.cross(sites,forces).sum(axis=0), {
        'surface_alpha_deg': alpha, 'surface_forces_body': forces,
        'surface_apparent_wind_body': apparent, 'surface_lift_body': lift,
        'surface_drag_body': drag, 'airpath_speed': float(speed[0]),
        'wind_speed': float(np.linalg.norm(W[0]))}
