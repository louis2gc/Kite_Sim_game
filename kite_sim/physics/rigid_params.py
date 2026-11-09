"""Uncalibrated rigid-body configuration; all body points are relative to COM."""
from dataclasses import dataclass
from functools import lru_cache
import numpy as np


@dataclass(frozen=True)
class KiteParams:
    area: float = 12.
    mass: float = 2.9
    line_length: float = 25.
    rho: float = 1.225
    gravity: float = 9.81
    k_ds: float = 1.5
    delta_m_max: float = .35
    motor_omega: float = 10.
    motor_zeta: float = 1.
    motor_rate_max: float = 1.05
    steering_delay: float = .10
    depower_rate: float = 2.
    wind_speed: float = 8.
    wind_heading_deg: float = 0.
    frame_heading_deg: float = 0.
    wind_z_ref: float = 4.
    wind_z_min: float = .5
    shear_exponent: float = .14
    turbulence_intensity: float = 0.
    turbulence_time_const: float = 3.
    wind_seed: int = 0
    aero_epsilon: float = 1e-8
    ground_height: float = 0.
    initial_phi_deg: float = 0.
    dt: float = .005
    initial_theta_deg: float = 70.
    initial_course_deg: float = 0.
    initial_speed: float | None = 0.
    # Euler angles ONLY initialize attitude, never integrate it.
    initial_pitch_deg: float = 0.
    initial_heading_deg: float = 0.
    width: float = 5.77
    height: float = 2.23
    relative_width: float = .9
    nose_distance: float = .2
    nose_mass_fraction: float = .47
    top_mass_fraction: float = .4
    # None estimates inertia from Fechner's four mass points. Replace with lab J.
    inertia: tuple | None = None
    attachments: tuple = ((-.5, 1.2, .8), (-.5, -1.2, .8))
    anchors: tuple = ((0., 0., 0.), (0., 0., 0.))
    line_gain: float = 1.
    side_area_ratio: float = .306
    alpha_zero_deg: float = 4.
    alpha_tip_deg: float = 10.
    depower_angle_deg: float = 10.
    # Effective differential side incidence; shape and inertia remain rigid.
    steering_twist_deg: float = 16.83
    alpha_cl: tuple = (-180., -160., -90., -20., -10., -5., 0., 20., 40., 90., 160., 180.)
    cl: tuple = (0., .5, 0., .08, .125, .15, .2, 1., 1., 0., -.5, 0.)
    alpha_cd: tuple = (-180., -170., -140., -90., -20., 0., 20., 90., 140., 170., 180.)
    cd: tuple = (.5, .5, .5, 1., .2, .1, .2, 1., .5, .5, .5)

    def __post_init__(self):
        arrays = {'inertia','attachments','anchors','alpha_cl','cl','alpha_cd','cd'}
        for name, value in vars(self).items():
            if name not in arrays and value is not None and not np.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in ('area','mass','line_length','rho','gravity','delta_m_max',
                     'motor_omega','motor_zeta','motor_rate_max','depower_rate','dt',
                     'wind_z_ref','wind_z_min','turbulence_time_const','aero_epsilon'):
            if getattr(self,name)<=0: raise ValueError(f'{name} must be positive')
        for name in ('wind_speed','steering_delay','k_ds','turbulence_intensity'):
            if getattr(self,name)<0: raise ValueError(f'{name} must be nonnegative')
        if not 0 < self.initial_theta_deg < 90:
            raise ValueError('Initial elevation must lie between 0 and 90 degrees')
        if self.initial_speed is not None and self.initial_speed<0:
            raise ValueError('Initial speed must be nonnegative')
        if int(self.wind_seed)!=self.wind_seed or self.wind_seed<0:
            raise ValueError('wind_seed must be a nonnegative integer')
        if not np.isclose(self.steering_delay/self.dt,round(self.steering_delay/self.dt),rtol=0,atol=1e-8):
            raise ValueError('steering_delay must be a whole number of physics steps')
        for name in ('attachments', 'anchors', 'inertia'):
            value = getattr(self, name)
            if value is not None:
                array = np.asarray(value, dtype=float)
                shape = (3, 3) if name == 'inertia' else (2, 3)
                if array.shape != shape or not np.all(np.isfinite(array)):
                    raise ValueError(f'{name} must have shape {shape} and finite values')
                object.__setattr__(self, name, tuple(map(tuple, array)))
        for name in ('width', 'height', 'relative_width', 'nose_distance', 'line_gain',
                     'side_area_ratio', 'nose_mass_fraction', 'top_mass_fraction'):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('relative_width','side_area_ratio','nose_mass_fraction','top_mass_fraction'):
            if getattr(self, name) >= 1:
                raise ValueError(f'{name} must be below one')
        for name in ('initial_pitch_deg','initial_heading_deg','alpha_zero_deg',
                     'alpha_tip_deg','depower_angle_deg','steering_twist_deg'):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f'{name} must be finite')
        if self.line_length <= .5*self.line_gain*self.delta_m_max:
            raise ValueError('Steering stroke could produce a nonpositive line length')
        for angles, values in (('alpha_cl','cl'), ('alpha_cd','cd')):
            a, v = np.asarray(getattr(self, angles)), np.asarray(getattr(self, values))
            if (a.ndim != 1 or v.shape != a.shape or len(a)<2 or
                not np.all(np.isfinite(a)) or not np.all(np.isfinite(v)) or
                np.any(np.diff(a)<=0) or a[0] != -180 or a[-1] != 180 or v[0] != v[-1]):
                raise ValueError('Polar tables must be finite, ordered, periodic over [-180,180]')
            object.__setattr__(self, angles, tuple(map(float, a)))
            object.__setattr__(self, values, tuple(map(float, v)))
        if min(self.cd)<0:
            raise ValueError('Drag coefficients must be nonnegative')
        if self.inertia is not None:
            J = np.asarray(self.inertia)
            moments = np.linalg.eigvalsh(J)
            if (not np.allclose(J, J.T, atol=1e-12) or min(moments)<=0 or
                max(moments)>sum(moments)-max(moments)+1e-10):
                raise ValueError('Inertia must be symmetric positive definite with physical principal moments')


@lru_cache(maxsize=32)
def geometry(p):
    """Fechner A/B/C/D geometry, with mass-weighted origin shifted to COM."""
    width = p.width*p.relative_width
    points = np.array([[p.nose_distance*width,0,0], [0,0,-p.height],
                       [0,width/2,0], [0,-width/2,0]], dtype=float)
    remaining = 1-p.nose_mass_fraction
    masses = p.mass*np.array([p.nose_mass_fraction, remaining*p.top_mass_fraction,
                              remaining*(1-p.top_mass_fraction)/2,
                              remaining*(1-p.top_mass_fraction)/2])
    points -= masses@points/p.mass
    J = sum(m*(np.dot(s,s)*np.eye(3)-np.outer(s,s)) for m,s in zip(masses,points))
    if p.inertia is not None:
        J = np.array(p.inertia)
    inv = np.linalg.inv(J)
    for a in (points, J, inv): a.setflags(write=False)
    return points, J, inv
