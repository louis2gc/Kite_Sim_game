"""Exercise the real input mapping, game loop, physics and screen projection."""
import os
os.environ.setdefault('SDL_VIDEODRIVER','dummy')
os.environ.setdefault('SDL_AUDIODRIVER','dummy')
from collections import defaultdict
from unittest.mock import patch
import unittest
import pygame
from kite_sim.config import load_params
from kite_sim.app.game import run
from kite_sim.app.render import Renderer
from kite_sim.app.camera import Camera
from kite_sim.physics.kite import source_position


class AppLoopTests(unittest.TestCase):
    def test_right_key_moves_on_screen_and_depower_does_not_stop_turn(self):
        for depower in (False,True):
            with self.subTest(depower=depower):
                trace=[]
                frame=0
                def keys():
                    nonlocal frame
                    held=defaultdict(bool)
                    if depower and frame<60:held[pygame.K_DOWN]=True
                    if 90<=frame<150:held[pygame.K_RIGHT]=True
                    if 150<=frame<210:held[pygame.K_LEFT]=True
                    frame+=1
                    return held
                original=Renderer.draw
                def draw(renderer,state,params,trail,command_s,command_d,paused):
                    trace.append((state,command_s,command_d))
                    original(renderer,state,params,trail,command_s,command_d,paused)
                with patch('pygame.key.get_pressed',side_effect=keys), \
                     patch('pygame.event.get',return_value=[]), \
                     patch.object(Renderer,'draw',draw):
                    p=load_params()
                    final=run(p,smoke_frames=240)
                self.assertFalse(final.stopped,final.stopped)
                self.assertGreater(final.time,3.9)
                self.assertEqual(trace[90][1],-.12)
                self.assertAlmostEqual(trace[149][0].actuators.delta_m,-.12,delta=.002)
                if depower:self.assertGreater(trace[89][0].actuators.depower,.99)
                camera=Camera((1150,820))
                before=camera.project(source_position(trace[89][0],p))
                after=camera.project(source_position(trace[149][0],p))
                self.assertGreater(after[0]-before[0],5.,'Right key must visibly move kite right within 1 second')
