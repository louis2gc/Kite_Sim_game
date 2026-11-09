"""Regressions for keyboard responsiveness and steering after depowering."""
from dataclasses import replace
import math
import unittest
from kite_sim.config import load_params
from kite_sim.app.game import KEYBOARD_STEERING_METRES
from kite_sim.physics.kite import initial_state,step,telemetry


class GameControlTests(unittest.TestCase):
    def test_keyboard_turn_release_and_reversal_at_each_depower_level(self):
        p=load_params()
        for depower in (0.,.25,.5,.75,1.):
            with self.subTest(depower=depower):
                s=initial_state(p)
                for _ in range(round(3/p.dt)):
                    s=step(s,0,depower,p)
                    self.assertFalse(s.stopped,s.stopped)
                start=s.phi
                for tick in range(round(6/p.dt)):
                    # Right for two seconds, left for two, then release.
                    t=tick*p.dt
                    command=-KEYBOARD_STEERING_METRES if t<2 else KEYBOARD_STEERING_METRES if t<4 else 0.
                    s=step(s,command,depower,p)
                    self.assertFalse(s.stopped,f'{depower=}, {t=}: {s.stopped}')
                    if tick==round(1/p.dt)-1:
                        # Detect movement in the correct direction within a second,
                        # including fully depowered flight; powered flight is faster.
                        self.assertLess(math.degrees(s.phi-start),-1.)
                        if depower==0:
                            self.assertLess(math.degrees(s.phi-start),-10.)
                self.assertGreater(min(telemetry(s,p)['line_tensions']),0.)
                self.assertAlmostEqual(s.actuators.delta_m,0.,places=6)

    def test_short_taps_while_depower_changes_in_gusts(self):
        p=replace(load_params(),turbulence_intensity=.1,wind_seed=7)
        s=initial_state(p)
        for tick in range(round(8/p.dt)):
            t=tick*p.dt
            # Alternating short taps and releases; sweep depower up and down.
            phase=int(t/.25)%4
            command=(0.,KEYBOARD_STEERING_METRES,0.,-KEYBOARD_STEERING_METRES)[phase]
            depower=min(t/2,1.) if t<4 else max(0.,1-(t-4)/2)
            s=step(s,command,depower,p)
            self.assertFalse(s.stopped,f'{t=}: {s.stopped}')
