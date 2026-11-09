"""Quaternion rigid kite driven by distributed Fechner aerodynamic loads.

Ground: X forward, Y left, Z up. Body: x nose, y along C-D, z down
through the canopy. Full COM inertia, two independently constrained lines.
"""
from dataclasses import dataclass, replace
import math
import numpy as np
from .rigid_params import geometry
from .quaternion import rotation, multiply, increment, normalize
from .constraints import accelerations, project, solve, ConstraintError
from .fechner_aero import loads
from .actuators import ActuatorState, delayed_command, actuator_derivatives
from .wind import WindState, initial_wind, advance_wind, wind_velocity
from .frames import tangent_frame


@dataclass(frozen=True)
class RigidState:
    position: tuple
    velocity: tuple
    quaternion: tuple
    omega: tuple
    actuators: ActuatorState
    wind: WindState
    time: float = 0.
    stopped: str = ''
    motor_command: float = 0.

    @property
    def theta(self):
        x,y,z = self.position
        return math.atan2(z,math.hypot(x,y))

    @property
    def phi(self):
        return math.atan2(self.position[1],self.position[0])

    @property
    def theta_dot(self):
        return float(tangent_frame(self.theta,self.phi)[:,0]@self.velocity/np.linalg.norm(self.position))

    @property
    def phi_dot(self):
        horizontal = math.hypot(*self.position[:2])
        # Only the display course/angle adapter is singular at zenith, not physics.
        return float(tangent_frame(self.theta,self.phi)[:,1]@self.velocity/max(horizontal,1e-12))


def position_ground(state,p): return np.array(state.position)


def source_position(state,p): return position_ground(state,p)*[1,-1,-1]


def position_enu(state,p):
    return rotation(increment([0,0,math.radians(p.frame_heading_deg)]))@state.position


def course(state): return math.atan2(math.cos(state.theta)*state.phi_dot,state.theta_dot)


def nose_source(state): return rotation(state.quaternion)[:,0]*[1,-1,-1]


def line_kinematics(dm,dmd,dmdd,p):
    gain = p.line_gain*np.array([-.5,.5])
    return p.line_length+gain*dm, gain*dmd, gain*dmdd


def attachment_positions(state,p):
    return np.asarray(state.position)+np.asarray(p.attachments)@rotation(state.quaternion).T


def motor_motion(state,p):
    a=state.actuators
    rate,acc,_=actuator_derivatives(a.delta_m,a.delta_m_dot,a.depower,state.motor_command,a.depower,p)
    if abs(a.delta_m)>=p.delta_m_max and a.delta_m*a.delta_m_dot>0: acc=0.
    return rate,acc


def dynamics(r,v,q,w,dm,dmd,d,dmdd,wind,p):
    force,torque,info = loads(r,v,q,w,dm,d,wind,p)
    # Gravity comes from V=mgz and is moved to RHS ONCE.
    force = force-np.array([0,0,p.mass*p.gravity])
    lengths,rates,accels = line_kinematics(dm,dmd,dmdd,p)
    _,J,Jinv = geometry(p)
    dv,dw,tensions,lam = accelerations(r,v,q,w,force,torque,lengths,rates,accels,
                                      p.attachments,p.anchors,p.mass,J,Jinv)
    a = np.asarray(r)+np.asarray(p.attachments)@rotation(q).T-np.asarray(p.anchors)
    info.update(force=float(np.linalg.norm(lam@a)),line_tensions=tensions,
                line_lengths=lengths, torque_body=torque, external_force_ground=force)
    return dv,dw,info


def state_vector(state):
    a = state.actuators
    return np.r_[state.position,state.velocity,state.quaternion,state.omega,a.delta_m,a.delta_m_dot,a.depower]


def stop_reason(state,p):
    if not np.all(np.isfinite(state_vector(state))): return 'Outside model: non-finite state'
    R = rotation(state.quaternion)
    sites = np.vstack([geometry(p)[0],p.attachments])
    if np.min((np.asarray(state.position)+sites@R.T)[:,2])<=p.ground_height:
        return 'Ground contact'
    try:
        _,_,info = dynamics(*[np.asarray(x) for x in (state.position,state.velocity,state.quaternion,state.omega)],
            state.actuators.delta_m,motor_motion(state,p)[0],state.actuators.depower,
            motor_motion(state,p)[1],state.wind,p)
        if min(info['line_tensions'])<0: return 'Outside model: slack bridle line'
    except (ConstraintError,ValueError,np.linalg.LinAlgError) as e:
        return f'Outside model: {e}'
    return ''


def initial_state(p):
    theta,phi = math.radians(p.initial_theta_deg),math.radians(p.initial_phi_deg)
    # At zero heading: body x=elevation tangent, y=azimuth tangent, z=-radial.
    q = multiply(increment([0,0,phi]),increment([0,-math.pi/2-theta,0]))
    q = multiply(q,increment([0,0,math.radians(p.initial_heading_deg)]))
    q = multiply(q,increment([0,math.radians(p.initial_pitch_deg),0]))
    r = p.line_length*np.array([math.cos(theta)*math.cos(phi),math.cos(theta)*math.sin(phi),math.sin(theta)])
    gamma = math.radians(p.initial_course_deg)
    v = (p.initial_speed or 0.)*(tangent_frame(theta,phi)@np.array([math.cos(gamma),math.sin(gamma),0]))
    lengths,rates,_ = line_kinematics(0,0,0,p)
    # Initialize position without changing the requested attitude.
    offsets=np.asarray(p.attachments)@rotation(q).T-np.asarray(p.anchors)
    for _ in range(20):
        a=r+offsets
        error=(np.einsum('ij,ij->i',a,a)-lengths**2)/2
        if np.max(np.abs(error))<1e-9: break
        r-=a.T@solve(a@a.T,error)
    r,v,q,w = project(r,v,q,np.zeros(3),lengths,rates,p.attachments,p.anchors,p.mass,geometry(p)[2])
    state = RigidState(tuple(r),tuple(v),tuple(q),tuple(w),
        ActuatorState(command_history=(0.,)*round(p.steering_delay/p.dt)),initial_wind(p.wind_seed))
    return replace(state,stopped=stop_reason(state,p))


def telemetry(state,p):
    result = dict(height=state.position[2],force=0.,airpath_speed=0.,
                  wind_speed=float(np.linalg.norm(wind_velocity(state.position[2],state.wind,p))),
                  speed=float(np.linalg.norm(state.velocity)),delta=p.line_gain*state.actuators.delta_m,
                  elevation_deg=math.degrees(state.theta),azimuth_deg=math.degrees(state.phi),
                  position=position_enu(state,p),course_deg=math.degrees(course(state)))
    try:
        _,_,info = dynamics(*[np.asarray(x) for x in (state.position,state.velocity,state.quaternion,state.omega)],
            state.actuators.delta_m,motor_motion(state,p)[0],state.actuators.depower,
            motor_motion(state,p)[1],state.wind,p)
        result.update(info)
    except (ValueError,np.linalg.LinAlgError): pass
    return result


def step(state,delta_m_ref,depower,p):
    if not math.isfinite(delta_m_ref) or not math.isfinite(depower):
        raise ValueError('Control inputs must be finite')
    if state.stopped: return state
    command = float(np.clip(delta_m_ref,-p.delta_m_max,p.delta_m_max))
    target = float(np.clip(depower,0,1))
    delayed,history = delayed_command(state.actuators,command)
    wind = advance_wind(state.wind,p)
    y = state_vector(state)
    def rhs(x):
        r,v,q,w = x[:3],x[3:6],x[6:10],x[10:13]
        ad = actuator_derivatives(*x[13:16],delayed,target,p)
        dmdd = ad[1]
        # At a hard stroke limit the actual line velocity and acceleration are zero.
        if abs(x[13])>=p.delta_m_max and x[13]*x[14]>0: dmdd=0.
        dv,dw,info = dynamics(r,v,q,w,x[13],ad[0],x[15],dmdd,wind,p)
        if min(info['line_tensions'])<0: raise ConstraintError('slack bridle line')
        if not np.all(np.isfinite(dv)) or not np.all(np.isfinite(dw)):
            raise ConstraintError('non-finite acceleration')
        return np.r_[v,dv,.5*multiply(q,np.r_[0,w]),dw,ad]
    try:
        k1=rhs(y);k2=rhs(y+p.dt*k1/2);k3=rhs(y+p.dt*k2/2);k4=rhs(y+p.dt*k3)
        x=y+p.dt*(k1+2*k2+2*k3+k4)/6
        dm=float(np.clip(x[13],-p.delta_m_max,p.delta_m_max))
        dmd=float(np.clip(x[14],-p.motor_rate_max,p.motor_rate_max))
        if abs(dm)>=p.delta_m_max and dm*dmd>0: dmd=0.
        lengths,rates,_=line_kinematics(dm,dmd,0,p)
        r,v,q,w=project(x[:3],x[3:6],x[6:10],x[10:13],lengths,rates,
                        p.attachments,p.anchors,p.mass,geometry(p)[2])
        actuator=ActuatorState(dm,dmd,float(np.clip(x[15],0,1)),history)
        result=RigidState(tuple(r),tuple(v),tuple(q),tuple(w),actuator,wind,state.time+p.dt,motor_command=delayed)
        reason=stop_reason(result,p)
        return replace(result,stopped=reason)
    except (ConstraintError,ValueError,np.linalg.LinAlgError) as e:
        return replace(state,stopped=f'Outside model: {e}')
