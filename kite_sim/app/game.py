"""Keyboard flight and a fixed rider camera, with optional headless smoke mode."""
import os
from time import perf_counter
from collections import deque
from pathlib import Path


KEYBOARD_STEERING_METRES = .12


def run(params, smoke_frames=0, screenshot=None):
    if smoke_frames:
        os.environ['SDL_VIDEODRIVER']='dummy'
        os.environ['SDL_AUDIODRIVER']='dummy'
    import pygame
    from kite_sim.physics.kite import initial_state,step,source_position
    from .render import Renderer
    pygame.display.init()
    pygame.font.init()
    screen=pygame.display.set_mode((1150,820))
    pygame.display.set_caption('Kite — a physics playground')
    renderer=Renderer(screen)
    clock=pygame.time.Clock()
    state=initial_state(params)
    trail=deque(maxlen=1800)
    accumulator=0.
    command_d=0.
    paused=False
    running=True
    frame=0
    start=perf_counter()
    try:
        while running:
            elapsed=1/60 if smoke_frames else min(clock.tick(60)/1000,.1)
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    running=False
                elif event.type==pygame.WINDOWFOCUSLOST:
                    paused=True
                    accumulator=0.
                elif event.type==pygame.KEYDOWN:
                    if event.key==pygame.K_ESCAPE:
                        running=False
                    elif event.key==pygame.K_SPACE:
                        paused=not paused
                        accumulator=0.
                    elif event.key==pygame.K_v:
                        renderer.instrument_mode=not renderer.instrument_mode
                    elif event.key==pygame.K_r:
                        state=initial_state(params)
                        trail.clear()
                        accumulator=0.
                        command_d=0.
                        paused=False
            keys=pygame.key.get_pressed()
            # Positive motor travel = left. Short taps are easier to control.
            command_s=min(KEYBOARD_STEERING_METRES,params.delta_m_max)*(float(keys[pygame.K_LEFT] or keys[pygame.K_a])-
                                                  float(keys[pygame.K_RIGHT] or keys[pygame.K_d]))
            if not paused and not state.stopped:
                dd=float(keys[pygame.K_DOWN] or keys[pygame.K_s])-float(keys[pygame.K_UP] or keys[pygame.K_w])
                command_d=max(0.,min(1.,command_d+dd*elapsed))
                accumulator+=elapsed
                while accumulator>=params.dt and not state.stopped:
                    after=step(state,command_s,command_d,params)
                    if after.stopped and not state.stopped:
                        print(f'Flight stopped at {after.time:.3f}s: {after.stopped}; '
                              f'steering={command_s:+.3f} m, depower={command_d:.0%}',flush=True)
                    state=after
                    accumulator-=params.dt
                trail.append(source_position(state,params))
            renderer.draw(state,params,trail,command_s,command_d,paused)
            if not renderer.instrument_mode:
                direction='LEFT' if command_s>0 else 'RIGHT' if command_s<0 else 'CENTRED'
                renderer.text(f'STEERING {direction} / FLIGHT {state.time:.1f} s / R TO RESTART',
                              (40,111),(25,65,80),renderer.small)
            pygame.display.flip()
            frame+=1
            if smoke_frames and frame>=smoke_frames:
                running=False
        cost=perf_counter()-start
        if screenshot:
            path=Path(screenshot)
            path.parent.mkdir(parents=True,exist_ok=True)
            pygame.image.save(screen,str(path))
        if smoke_frames:
            print(f'App smoke: {frame} frames, {state.time:.3f} simulated seconds; stop={state.stopped or "none"}')
            print(f'Headless frame cost (physics + drawing): {cost*1000/frame:.2f} ms average')
        return state
    finally:
        pygame.quit()
