"""Motor displacement in metres, second-order servo, rate/stroke limits + delay.

Fagiano 2014 Eq. 20 for the servo; rate limits and transport delay are the simulator’s
extensions. Depower remains a rate-limited normalized engineering model.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class ActuatorState:
    delta_m: float = 0.0
    delta_m_dot: float = 0.0
    depower: float = 0.0
    command_history: tuple[float, ...] = ()


def delayed_command(state, command):
    if not state.command_history:
        return command, ()
    return state.command_history[0], state.command_history[1:]+(command,)


def actuator_derivatives(dm, dmd, depower, command, target_depower, p):
    ref = max(-p.delta_m_max, min(p.delta_m_max, command))
    velocity = max(-p.motor_rate_max, min(p.motor_rate_max, dmd))
    acceleration = (p.motor_omega**2*(ref-dm)
                    -2*p.motor_zeta*p.motor_omega*dmd)
    if (dmd >= p.motor_rate_max and acceleration > 0) or (
            dmd <= -p.motor_rate_max and acceleration < 0):
        acceleration = 0.0
    if (dm >= p.delta_m_max and velocity > 0) or (
            dm <= -p.delta_m_max and velocity < 0):
        velocity = 0.0
    dd = max(-p.depower_rate, min(p.depower_rate, (target_depower-depower)/p.dt))
    return velocity, acceleration, dd
