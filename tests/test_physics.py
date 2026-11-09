"""Mechanics invariants and integration checks, standalone checks."""
from dataclasses import replace
import unittest
import numpy as np
from kite_sim.config import load_params
from kite_sim.physics.kite import initial_state,step,telemetry,nose_source
from kite_sim.physics.quaternion import rotation,multiply,increment
from kite_sim.physics.rigid_params import geometry
from kite_sim.physics.fechner_aero import loads,polar
from kite_sim.physics.constraints import accelerations,system,project,ConstraintError
from kite_sim.physics.kite import line_kinematics,attachment_positions
from kite_sim.app.canopy import canopy_vertices


class RigidPhysicsTests(unittest.TestCase):
    def setUp(self): self.p=load_params()

    def test_rotation_conventions_and_quaternion_derivative(self):
        q=increment([.3,-.4,.8]);R=rotation(q)
        np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-14)
        np.testing.assert_allclose(rotation(-q),R,atol=1e-14)
        self.assertAlmostEqual(np.linalg.det(R),1)
        w=np.array([1.,2.,3.]);s=np.array([.4,.2,-1.])
        eps=1e-7
        derivative=(rotation(q+.5*eps*multiply(q,np.r_[0,w]))@s-R@s)/eps
        np.testing.assert_allclose(derivative,R@np.cross(w,s),atol=1e-6)

    def test_body_heading_is_independent_of_course_and_renderer_uses_attitude(self):
        s=initial_state(self.p)
        other=replace(s,velocity=(5.,6.,7.))
        np.testing.assert_array_equal(nose_source(s),nose_source(other))
        tilted=replace(s,quaternion=tuple(multiply(s.quaternion,increment([.3,.2,.1]))))
        self.assertGreater(np.linalg.norm(nose_source(s)-nose_source(tilted)),.1)
        _,lead,_=canopy_vertices(s,self.p)
        _,lead2,_=canopy_vertices(tilted,self.p)
        self.assertGreater(np.linalg.norm(np.array(lead)-lead2),.1)

    def test_fechners_loads_sum_and_local_velocity(self):
        s=initial_state(self.p);w=np.array([.2,-.3,.5]);r=np.array(s.position);v=np.array([1,2,3])
        force,torque,info=loads(r,v,s.quaternion,w,0,0,s.wind,self.p)
        sites=geometry(self.p)[0][1:];R=rotation(s.quaternion)
        np.testing.assert_allclose(force,R@info['surface_forces_body'].sum(axis=0))
        np.testing.assert_allclose(torque,np.cross(sites,info['surface_forces_body']).sum(axis=0))
        _,_,rest=loads(r,v,s.quaternion,np.zeros(3),0,0,s.wind,self.p)
        np.testing.assert_allclose(info['surface_apparent_wind_body']-rest['surface_apparent_wind_body'],
                                   -np.cross(w,sites),atol=1e-14)
        va=info['surface_apparent_wind_body']
        np.testing.assert_allclose(np.einsum('ij,ij->i',info['surface_lift_body'],va),0,atol=1e-11)
        self.assertTrue(np.all(np.einsum('ij,ij->i',info['surface_drag_body'],va)>=0))
        _,_,symmetric=loads(r,np.zeros(3),s.quaternion,np.zeros(3),0,0,s.wind,self.p)
        self.assertAlmostEqual(symmetric['surface_forces_body'][1,1],-symmetric['surface_forces_body'][2,1])
        self.assertAlmostEqual(float(polar(20,self.p)[0]),1.)
        self.assertAlmostEqual(float(polar(0,self.p)[1]),.1)

    def test_augmented_equations_with_full_inertia_and_moving_lines(self):
        p=self.p;r=np.array([1.,2.,24.]);v=np.array([.3,-.2,.1]);q=increment([.2,.1,-.3])
        w=np.array([.3,.4,-.2]);F=np.array([12.,3.,80.]);tau=np.array([.3,2.,-1.])
        J=np.array([[3.,.2,.1],[.2,4.,-.1],[.1,-.1,5.]]);inv=np.linalg.inv(J)
        L=np.array([25.,24.9]);Ld=np.array([.1,-.2]);Ldd=np.array([.3,-.4])
        dv,dw,T,lam=accelerations(r,v,q,w,F,tau,L,Ld,Ldd,p.attachments,p.anchors,p.mass,J,inv)
        R,a,h,M=system(r,q,p.attachments,p.anchors,p.mass,inv)
        np.testing.assert_allclose(p.mass*dv,F-lam@a,atol=1e-12)
        np.testing.assert_allclose(J@dw+np.cross(w,J@w),tau-lam@h,atol=1e-12)
        attachment_velocity=v+np.cross(w,p.attachments)@R.T
        attachment_acceleration=dv+(np.cross(dw,p.attachments)+np.cross(w,np.cross(w,p.attachments)))@R.T
        second=np.einsum('ij,ij->i',attachment_velocity,attachment_velocity)+np.einsum('ij,ij->i',a,attachment_acceleration)-Ld**2-L*Ldd
        np.testing.assert_allclose(second,0,atol=1e-11)
        np.testing.assert_allclose(T,lam*L)

    def test_constraints_do_correct_work(self):
        p=self.p;s=initial_state(p);J,inv=geometry(p)[1:]
        L,Ld,Ldd=line_kinematics(.01,.2,-.3,p)
        r,v,q,w=project(s.position,[1,2,3],s.quaternion,[.2,-.3,.4],L,Ld,
                        p.attachments,p.anchors,p.mass,inv)
        F=np.array([1.,2.,100.]);tau=np.array([3.,-4.,5.])
        dv,dw,_,lam=accelerations(r,v,q,w,F,tau,L,Ld,Ldd,p.attachments,p.anchors,p.mass,J,inv)
        power=p.mass*v@dv+w@J@dw
        self.assertAlmostEqual(power,F@v+tau@w-np.sum(lam*L*Ld),places=10)

    def test_static_two_lines_share_weight(self):
        b=np.array([[0,1,0],[0,-1,0.]])
        anchors=np.array([[0,1,0],[0,-1,0.]])
        # Suspend body below anchors: equal upward tension balances gravity.
        dv,dw,T,_=accelerations(np.array([0,0,-10.]),np.zeros(3),[1,0,0,0],np.zeros(3),
            np.array([0,0,-20.]),np.zeros(3),[10,10],[0,0],[0,0],b,anchors,2,np.eye(3),np.eye(3))
        np.testing.assert_allclose(dv,0,atol=1e-13)
        np.testing.assert_allclose(dw,0,atol=1e-13)
        np.testing.assert_allclose(T,[10,10])

    def assert_constraints(self,s,p):
        L,Ld,_=line_kinematics(s.actuators.delta_m,s.actuators.delta_m_dot,0,p)
        a=attachment_positions(s,p)-p.anchors
        self.assertLess(np.max(np.abs(np.linalg.norm(a,axis=1)-L)),1e-9)
        va=s.velocity+np.cross(s.omega,p.attachments)@rotation(s.quaternion).T
        np.testing.assert_allclose(np.einsum('ij,ij->i',a,va),L*Ld,atol=1e-9)
        self.assertAlmostEqual(np.linalg.norm(s.quaternion),1.,places=13)

    def test_flight_steering_delay_constraints_and_mirror_symmetry(self):
        positive=negative=initial_state(self.p)
        for i in range(400):
            positive=step(positive,.03,0,self.p)
            negative=step(negative,-.03,0,self.p)
            self.assertFalse(positive.stopped)
            self.assertFalse(negative.stopped)
            self.assert_constraints(positive,self.p)
            if i<round(self.p.steering_delay/self.p.dt):
                self.assertEqual(positive.actuators.delta_m,0)
        np.testing.assert_allclose(positive.position,np.array(negative.position)*[1,-1,1],atol=1e-9)
        self.assertGreater(positive.phi,0) # Positive motor travel remains left steering.
        self.assertGreater(positive.actuators.delta_m,0)

    def test_gravity_once(self):
        s=initial_state(self.p)
        force,_,_=loads(s.position,s.velocity,s.quaternion,s.omega,0,0,s.wind,self.p)
        np.testing.assert_allclose(telemetry(s,self.p)['external_force_ground'],force-[0,0,self.p.mass*self.p.gravity])

    def test_timestep_convergence_and_parked_flight(self):
        end=[]
        for dt in (.01,.005,.0025,.00125):
            p=replace(self.p,dt=dt);s=initial_state(p)
            for _ in range(round(1/dt)):s=step(s,.03,0,p)
            self.assertFalse(s.stopped)
            end.append(np.r_[s.position,s.velocity,s.quaternion,s.omega])
        coarse=np.linalg.norm(end[0]-end[1]);fine=np.linalg.norm(end[1]-end[2])
        # Piecewise-linear polars and unsigned incidence contain kinks; do
        # not demand fourth-order ratios across those crossings.
        finer=np.linalg.norm(end[2]-end[3])
        self.assertLess(fine,coarse)
        self.assertLess(finer,fine)
        self.assertLess(fine,.002)
        s=initial_state(self.p)
        for _ in range(2000):s=step(s,0,0,self.p)
        self.assertFalse(s.stopped)
        self.assert_constraints(s,self.p)

    def test_invalid_geometry_inertia_slack_and_ground_are_explicit(self):
        with self.assertRaises(ValueError):replace(self.p,inertia=[[1,0,0],[0,-1,0],[0,0,1]])
        with self.assertRaises(ValueError):replace(self.p,inertia=[[1,0,0],[0,1,0],[0,0,4]])
        with self.assertRaises(ValueError):replace(self.p,cd=[-1]*len(self.p.cd))
        with self.assertRaises((ConstraintError,np.linalg.LinAlgError)):
            initial_state(replace(self.p,attachments=((0,0,0),(0,0,0))))
        no_wind=initial_state(replace(self.p,wind_speed=0))
        self.assertIn('slack',no_wind.stopped)
        from kite_sim.physics.kite import stop_reason
        self.assertEqual(stop_reason(replace(initial_state(self.p),position=(0,0,0)),self.p),'Ground contact')
        with self.assertRaises(ValueError):step(initial_state(self.p),np.nan,0,self.p)

    def test_seeded_flight_is_deterministic_and_state_is_immutable(self):
        p=replace(self.p,turbulence_intensity=.1,wind_seed=4)
        first=second=initial_state(p)
        original=first
        for _ in range(100):
            first=step(first,.02,.05,p)
            second=step(second,.02,.05,p)
        self.assertEqual(first,second)
        self.assertFalse(first.stopped)
        self.assertEqual(original.time,0.)
        self.assertEqual(original.actuators.delta_m,0.)
