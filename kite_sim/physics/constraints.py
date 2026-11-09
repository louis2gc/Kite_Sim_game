"""Two prescribed-length, massless taut lines; COM forces and body moments.

Constraint g_i=(a_i.a_i-L_i²)/2=0, physical force -lambda_i*a_i.
No compression, elasticity, slack recovery or hidden constraint springs.
"""
import numpy as np
from .quaternion import rotation, multiply, increment, normalize


class ConstraintError(ValueError):
    pass


def system(position, q, attachments, anchors, mass, inverse_inertia):
    R = rotation(q)
    a = np.asarray(position)+np.asarray(attachments)@R.T-np.asarray(anchors)
    h = np.cross(attachments, a@R)
    matrix = a@a.T/mass+h@inverse_inertia@h.T
    return R, a, h, matrix


def solve(matrix, rhs):
    # Two independent scalar constraints are required, including at initialization.
    if not np.all(np.isfinite(matrix)) or np.linalg.cond(matrix)>1e10:
        raise ConstraintError('dependent or ill-conditioned bridle constraints')
    return np.linalg.solve(matrix, rhs)


def accelerations(position, velocity, q, omega, force, torque, lengths, length_rates,
                  length_accelerations, attachments, anchors, mass, inertia, inverse_inertia):
    R, a, h, matrix = system(position,q,attachments,anchors,mass,inverse_inertia)
    omega = np.asarray(omega)
    k = torque-np.cross(omega, inertia@omega)
    site_velocity = velocity+np.cross(omega,attachments)@R.T
    centrifugal = np.cross(omega,np.cross(omega,attachments))@R.T
    rhs = (np.einsum('ij,ij->i',site_velocity,site_velocity)+a@force/mass+
           h@inverse_inertia@k+np.einsum('ij,ij->i',a,centrifugal)-
           np.asarray(length_rates)**2-np.asarray(lengths)*length_accelerations)
    lam = solve(matrix,rhs)
    acceleration = (force-lam@a)/mass
    angular_acceleration = inverse_inertia@(k-lam@h)
    return acceleration, angular_acceleration, lam*np.asarray(lengths), lam


def project(position, velocity, q, omega, lengths, length_rates,
            attachments, anchors, mass, inverse_inertia):
    """Mass-weighted Newton position projection, then velocity projection.

    Quaternion updates use body rotation increments. This controls numerical
    drift; it is not a substitute for solving acceleration constraints.
    """
    r, v, q, w = (np.array(x,dtype=float,copy=True) for x in (position,velocity,q,omega))
    lengths = np.asarray(lengths)
    for _ in range(12):
        _,a,h,matrix = system(r,q,attachments,anchors,mass,inverse_inertia)
        error = (np.einsum('ij,ij->i',a,a)-lengths**2)/2
        if np.max(np.abs(error)/lengths)<1e-10: break
        correction = solve(matrix,error)
        r -= correction@a/mass
        q = normalize(multiply(q,increment(-inverse_inertia@(correction@h))))
    else:
        raise ConstraintError('bridle position projection did not converge')
    _,a,h,matrix = system(r,q,attachments,anchors,mass,inverse_inertia)
    correction = solve(matrix,a@v+h@w-lengths*np.asarray(length_rates))
    v -= correction@a/mass
    w -= inverse_inertia@(correction@h)
    return r,v,normalize(q),w
