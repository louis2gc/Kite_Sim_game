"""Orthographic front view of the wind hemisphere, looking downwind."""
import math
import pygame
from kite_sim.physics.kite import source_position, nose_source, telemetry

BG = (13, 22, 35)
INK = (228, 238, 246)
MUTED = (143, 165, 186)
CYAN = (89, 224, 221)
ORANGE = (255, 183, 103)


class Renderer:
    def __init__(self, surface):
        self.surface = surface
        self.font = pygame.font.SysFont('Helvetica', 19)
        self.small = pygame.font.SysFont('Helvetica', 15)
        self.title = pygame.font.SysFont('Helvetica', 31, bold=True)
        self.value = pygame.font.SysFont('Helvetica', 30)
        self.origin = (408, 622)
        self.radius = 335

    def text(self, text, xy, color=INK, font=None):
        self.surface.blit((font or self.font).render(text, True, color), xy)

    def project(self, position, length):
        return (round(self.origin[0]+self.radius*position[1]/length),
                round(self.origin[1]+self.radius*position[2]/length))

    def draw(self, state, params, trail, command_s, command_d, paused):
        s = self.surface
        s.fill(BG)
        self.text('KITE / FLIGHT VIEW', (36, 28), font=self.title)
        self.text('Quaternion rigid body · two taut lines', (38, 73), MUTED)
        status = state.stopped or ('Paused' if paused else 'Flying')
        self.text(status, (38, 114), ORANGE if state.stopped else CYAN)
        ox, oy = self.origin
        r = self.radius
        # Front projection: equal polar-angle rings and meridians about downwind.
        sky = [(ox-r, oy)] + [(round(ox+r*math.cos(a)), round(oy-r*math.sin(a)))
                             for a in [i*math.pi/100 for i in range(101)]]
        pygame.draw.polygon(s, (20, 37, 55), sky)
        for angle in (15, 30, 45, 60, 75, 90):
            rr = r*math.sin(math.radians(angle))
            points = [(round(ox+rr*math.cos(i*math.pi/100)),
                       round(oy-rr*math.sin(i*math.pi/100))) for i in range(101)]
            pygame.draw.lines(s, (43, 65, 84), False, points, 1)
        for angle in range(-60, 90, 30):
            a = math.radians(angle)
            pygame.draw.line(s, (43, 65, 84), (ox, oy),
                             (round(ox+r*math.sin(a)), round(oy-r*math.cos(a))))
        pygame.draw.line(s, MUTED, (ox-r-14, oy), (ox+r+14, oy), 2)
        self.text('ZENITH', (ox-28, oy-r-27), MUTED, self.small)
        self.text('LEFT', (ox-r-2, oy+16), MUTED, self.small)
        self.text('DOWNWIND', (ox-43, oy+16), MUTED, self.small)
        self.text('RIGHT', (ox+r-38, oy+16), MUTED, self.small)
        self.text('Front view · looking downwind', (38, 191), MUTED, self.small)
        if len(trail) > 1:
            pygame.draw.lines(s, (57, 130, 146), False,
                              [self.project(pos, params.line_length) for pos in trail], 2)
        pos = source_position(state, params)
        point = self.project(pos, params.line_length)
        pygame.draw.line(s, (75, 98, 118), self.origin, point, 1)
        pygame.draw.circle(s, INK, self.origin, 4)
        nose = nose_source(state)
        dx, dy = nose[1], nose[2]
        magnitude = math.hypot(dx, dy)
        if magnitude > 1e-8:
            dx, dy = dx/magnitude, dy/magnitude
        else:
            dx, dy = 0.0, -1.0
        px, py = point
        polygon = [(px+dx*17, py+dy*17), (px-dx*10-dy*14, py-dy*10+dx*14),
                   (px-dx*5, py-dy*5), (px-dx*10+dy*14, py-dy*10-dx*14)]
        pygame.draw.polygon(s, ORANGE if state.stopped else CYAN, polygon)
        pygame.draw.polygon(s, INK, polygon, 1)
        # Telemetry panel.
        pygame.draw.rect(s, (21, 34, 49), (804, 151, 310, 500), border_radius=16)
        info = telemetry(state, params)
        self.text('TETHER PULL', (829, 173), MUTED, self.small)
        self.text(f'{info["force"]:,.0f} N', (828, 197), font=self.value)
        max_force=1000. # Gauge display scale [N], not a force or safety limit.
        pygame.draw.rect(s, (43, 61, 79), (830, 240, 258, 6), border_radius=3)
        pygame.draw.rect(s, CYAN, (830, 240, round(258*max(0,min(1, info['force']/max(max_force, 1)))), 6))
        rows = [('Elevation', f'{info["elevation_deg"]:.1f}°'),
                ('Azimuth', f'{info["azimuth_deg"]:+.1f}°'),
                ('Height', f'{info["height"]:.1f} m'),
                ('Wind', f'{info["wind_speed"]:.1f} m/s'),
                ('Line forces', ' / '.join(f'{t:.0f}' for t in info['line_tensions'])+' N') if 'line_tensions' in info else ('Line', f'{params.line_length:.0f} m'),
                ('Flight time', f'{state.time:.1f} s')]
        for index, (label, value) in enumerate(rows):
            y = 270+index*32
            self.text(label, (830, y), MUTED)
            self.text(value, (990, y))
        self.text(f'MOTOR {state.actuators.delta_m:+.3f} m', (830, 477), MUTED, self.small)
        pygame.draw.line(s, MUTED, (840, 514), (1080, 514), 3)
        pygame.draw.line(s, INK, (960, 506), (960, 522), 1)
        pygame.draw.circle(s, CYAN, (round(960-120*(state.actuators.delta_m/params.delta_m_max)), 514), 7)
        pygame.draw.circle(s, ORANGE, (round(960-120*command_s/params.delta_m_max), 514), 10, 2)
        self.text(f'DEPOWER  {state.actuators.depower:.0%}', (830, 547), MUTED, self.small)
        pygame.draw.rect(s, (43, 61, 79), (840, 578, 240, 7))
        pygame.draw.rect(s, CYAN, (840, 578, round(240*state.actuators.depower), 7))
        self.text(f'Target {command_d:.0%}', (830, 603), MUTED, self.small)
        self.text('Left / Right  steer    Up / Down  power in / depower', (38, 692))
        self.text('Space pause   R reset   Esc quit', (38, 726), MUTED)
        self.text('Experimental physics · static polars · no slack-line recovery',
                  (38, 774), MUTED, self.small)
        if state.stopped:
            self.text('Press R to launch a fresh flight.', (38, 151), ORANGE)
