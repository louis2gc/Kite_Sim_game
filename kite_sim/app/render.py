"""Cached outdoor scenery and a projected kite mesh; no 3D engine required."""
import math
import random
import numpy as np
import pygame
from .camera import Camera
from .canopy import canopy_vertices
from .instrument_view import Renderer as InstrumentRenderer
from kite_sim.physics.kite import source_position, nose_source, telemetry
from kite_sim.physics.kite import attachment_positions

INK = (239, 248, 250)
MUTED = (175, 202, 209)


class Renderer:
    def __init__(self, surface):
        self.surface = surface
        self.camera = Camera(surface.get_size())
        self.font = pygame.font.SysFont('Helvetica', 19)
        self.small = pygame.font.SysFont('Helvetica', 15)
        self.title = pygame.font.SysFont('Helvetica', 26, bold=True)
        self.instrument = InstrumentRenderer(surface)
        self.instrument_mode = False
        self.background = self.make_background()
        self.hud = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
        pygame.draw.rect(self.hud, (11, 36, 47, 205), (22, 20, 1106, 76), border_radius=14)
        pygame.draw.rect(self.hud, (11, 36, 47, 210), (22, 658, 220, 137), border_radius=12)
        pygame.draw.rect(self.hud, (11, 36, 47, 210), (827, 697, 301, 98), border_radius=12)

    def text(self, text, xy, color=INK, font=None):
        self.surface.blit((font or self.font).render(text, True, color), xy)

    def make_background(self):
        """Draw once. Clouds stay fixed in the world because the camera is fixed."""
        size = self.surface.get_size()
        bg = pygame.Surface(size).convert()
        w, h = size
        horizon = round(self.camera.horizon)
        for y in range(h):
            if y < horizon:
                t = y/horizon
                color = tuple(round(a+(b-a)*t) for a, b in zip((67, 132, 188), (211, 234, 231)))
            else:
                t = (y-horizon)/(h-horizon)
                color = tuple(round(a+(b-a)*t) for a, b in zip((91, 164, 177), (22, 97, 122)))
            pygame.draw.line(bg, color, (0, y), (w, y))
        # Soft cloud banks rendered at reduced resolution then smoothly enlarged.
        cloud_layer = pygame.Surface((w//2, h//2), pygame.SRCALPHA)
        rng = random.Random(17)
        for az, elev, scale in [(-55, 30, 1.1), (-23, 53, 0.65), (34, 61, 0.8),
                                 (62, 27, 1.0), (17, 12, 0.6), (-34, 9, 0.5)]:
            center = self.camera.direction(az, elev)
            if center is None:
                continue
            cx, cy = center[0]/2, center[1]/2
            for i in range(15):
                x = cx+(i-7)*9*scale
                y = cy+rng.uniform(-11, 9)*scale
                rw = rng.uniform(25, 48)*scale
                rh = rng.uniform(9, 23)*scale
                pygame.draw.ellipse(cloud_layer, (249, 253, 248, 120),
                                    (x-rw/2, y-rh/2, rw, rh))
                pygame.draw.ellipse(cloud_layer, (255, 255, 252, 90),
                                    (x-rw/2, y-rh/2-3, rw, rh*.65))
        bg.blit(pygame.transform.smoothscale(cloud_layer, size), (0, 0))
        pygame.draw.line(bg, (217, 238, 228), (0, horizon), (w, horizon), 2)
        # Distant coast and tiny sailboat provide fixed scale/horizon references.
        pygame.draw.polygon(bg, (105, 154, 164),
                            [(0, horizon), (0, horizon-9), (53, horizon-14),
                             (96, horizon-6), (140, horizon-4), (195, horizon)])
        bx, by = 924, horizon+10
        pygame.draw.polygon(bg, (244, 242, 225), [(bx, by-33), (bx, by-3), (bx-16, by-3)])
        pygame.draw.line(bg, (54, 103, 121), (bx, by-35), (bx, by+2))
        pygame.draw.line(bg, (54, 103, 121), (bx-19, by+2), (bx+6, by+2), 2)
        for _ in range(140):
            y = rng.randrange(horizon+4, h)
            x = rng.randrange(w)
            length = int(5+(y-horizon)*rng.uniform(.08, .4))
            pygame.draw.line(bg, (130, 188, 193), (x, y), (x+length, y))
        return bg

    def kite(self, state, params, bar_left, bar_right):
        center, leading_world, trailing_world = canopy_vertices(state, params)
        leading = self.camera.project_object(center, leading_world)
        trailing = self.camera.project_object(center, trailing_world)
        if any(p is None for p in leading+trailing):
            return
        ends=self.camera.project_object(center,attachment_positions(state,params)*[1,-1,-1])
        lines=list(zip((bar_left,bar_right),ends))
        # Short branches illustrate effective bridle points; only the long lines
        # are physical constraints in the model.
        for end,index in zip(ends,(12,4)):
            if end is not None:
                lines.extend([(end,leading[index]),(end,trailing[index])])
        for base, target in lines:
            if target is None: continue
            pygame.draw.aaline(self.surface, (225, 237, 232), base, target)
        palette = [(16, 64, 74), (232, 95, 52), (248, 128, 64), (255, 179, 96)]
        for i in range(16):
            color = palette[0] if i in (0, 1, 7, 8, 14, 15) else palette[1+(i%3)]
            polygon = [leading[i], leading[i+1], trailing[i+1], trailing[i]]
            pygame.draw.polygon(self.surface, color, polygon)
            pygame.draw.aalines(self.surface, (242, 197, 154), True, polygon)
        pygame.draw.lines(self.surface, (20, 60, 69), False, leading, 4)
        pygame.draw.aalines(self.surface, (255, 219, 165), False, leading)
        pygame.draw.aalines(self.surface, (27, 68, 75), False, trailing)

    def locator(self, state, params):
        """Small hemisphere map keeps location clear at the edge of the camera."""
        ox, oy, r = 131, 763, 72
        points = [(ox+r*math.cos(i*math.pi/60), oy-r*math.sin(i*math.pi/60))
                  for i in range(61)]
        pygame.draw.aalines(self.surface, (131, 178, 190), False, points)
        pygame.draw.line(self.surface, (131, 178, 190), (ox-r, oy), (ox+r, oy))
        pygame.draw.line(self.surface, (76, 122, 139), (ox, oy), (ox, oy-r))
        pos = source_position(state, params)/params.line_length
        p = (round(ox+r*pos[1]), round(oy+r*pos[2]))
        pygame.draw.line(self.surface, (123, 169, 182), (ox, oy), p)
        pygame.draw.circle(self.surface, (255, 171, 93), p, 5)
        self.text('YOUR WIND WINDOW', (40, 670), MUTED, self.small)
        self.text('LEFT         FRONT         RIGHT', (41, 772), MUTED, self.small)

    def draw(self, state, params, trail, command_s, command_d, paused):
        if self.instrument_mode:
            self.instrument.draw(state, params, trail, command_s, command_d, paused)
            self.text('V  rider view', (825, 725))
            return
        s = self.surface
        s.blit(self.background, (0, 0))
        steer = state.actuators.delta_m/params.delta_m_max
        depower = state.actuators.depower
        # Bar moves away on depower; tilt follows actual rate-limited input.
        bar_y = 754-55*depower
        bar_left = (465, round(bar_y-28*steer))
        bar_right = (685, round(bar_y+28*steer))
        self.kite(state, params, bar_left, bar_right)
        # Rider hands / bar are foreground depth cues, not physics geometry.
        pygame.draw.line(s, (205, 163, 130), (405, 850), bar_left, 32)
        pygame.draw.line(s, (205, 163, 130), (745, 850), bar_right, 32)
        pygame.draw.line(s, (29, 55, 62), bar_left, bar_right, 14)
        middle = (575, round(bar_y))
        pygame.draw.line(s, (221, 75, 60), bar_left, middle, 10)
        pygame.draw.line(s, (74, 174, 193), middle, bar_right, 10)
        for hand in (bar_left, bar_right):
            pygame.draw.circle(s, (231, 185, 149), hand, 14)
        pygame.draw.line(s, (37, 65, 70), middle, (575, 820), 3)
        s.blit(self.hud, (0, 0))
        self.text('KITE / COAST', (41, 31), font=self.title)
        self.text('Rigid kite · V instruments · Esc quit', (41, 64), MUTED, self.small)
        info = telemetry(state, params)
        self.text(f'{info["elevation_deg"]:.0f}° UP     {info["azimuth_deg"]:+.0f}° SIDE', (471, 37))
        self.text(f'{info["height"]:.1f} m HIGH   /   {params.line_length:.0f} m LINES', (471, 65), MUTED, self.small)
        self.text(f'{info["force"]:,.0f} N PULL', (883, 35))
        self.text(f'{info["wind_speed"]:.1f} m/s WIND    {depower:.0%} DEPOWER', (868, 65), MUTED, self.small)
        self.locator(state, params)
        self.text('Left / Right  steer', (845, 710), font=self.small)
        self.text('Up / Down  power / depower', (845, 733), font=self.small)
        self.text('Space pause   R reset   Esc quit', (845, 766), MUTED, self.small)
        # Camera never follows the kite: clouds remain a reliable reference.
        p = self.camera.project(source_position(state, params))
        if p is None or not (30 < p[0] < 1120 and 108 < p[1] < 650):
            direction = np.array(p)-self.camera.center if p else np.array([math.copysign(1, info['azimuth_deg']), 0])
            direction /= max(np.linalg.norm(direction), 1e-8)
            # Intersect a rectangle around the scene with the direction ray.
            scale = min(520/max(abs(direction[0]), 1e-8), 270/max(abs(direction[1]), 1e-8))
            tip = self.camera.center+direction*scale
            side = np.array([-direction[1], direction[0]])
            pygame.draw.polygon(s, (255, 146, 65),
                                [tip, tip-direction*20+side*9, tip-direction*20-side*9])
            self.text('Kite beyond view — follow the orange marker', (353, 117), (22, 62, 79), self.small)
        if state.stopped or paused:
            text = state.stopped+'  /  R to reset' if state.stopped else 'Paused  /  Space to resume'
            rect = pygame.Rect(260, 563, 630, 46)
            pygame.draw.rect(s, (18, 51, 65), rect, border_radius=10)
            self.text(text, (279, 576))
