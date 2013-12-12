#!/usr/bin/python
# -*- coding: utf-8 -*-

import pygame
import pygame.gfxdraw
from tunnel import SFTunnel
from agent import SFAgent
from pygame.locals import QUIT
import numpy as np

# Graphics setup

WINDOW_WIDTH = 1366
WINDOW_HEIGHT = 768
BLACK = pygame.Color(0, 0, 0)
WHITE = pygame.Color(255, 255, 255)
FAIR_BLUE = pygame.Color(51, 102, 255)
COLORADO_RED = pygame.Color(255, 102, 51)
RADIUS = 10
PATH_LENGTH = 200


def init_bottleneck():

    agents = []
    segments = [SFTunnel.Segment([100, WINDOW_HEIGHT / 2],
                [WINDOW_WIDTH / 2 - 50, WINDOW_HEIGHT / 2]),
                SFTunnel.Segment([WINDOW_WIDTH / 2 + 50, WINDOW_HEIGHT
                / 2], [WINDOW_WIDTH - 100, WINDOW_HEIGHT / 2])]

    # SFTunnel.Segment([100, 0], [100, WINDOW_HEIGHT/2]),
    # SFTunnel.Segment([WINDOW_WIDTH-100, 0],[WINDOW_WIDTH-100, WINDOW_HEIGHT/2])

    tunnel = SFTunnel(time_step=0.1, segments=segments)

    dest = [WINDOW_WIDTH / 2, WINDOW_HEIGHT - 100]

    for i in range(3):
        for j in range(3):
            pos = [600 + 80 * i, 200 + 40 * j]

            params = {
                'tunnel': tunnel,
                'position': pos,
                'destination': dest,
                'radius': RADIUS,
                'assumed_speed': RADIUS * 1.3,
                'max_speed': RADIUS * 2.6,
                'velocity': [8, 8],
                }

            agent = SFAgent(**params)
            agents.append(agent)

    return (segments, agents)


def init_maze():

    agents = []
    segments = [
        SFTunnel.Segment([0, 100], [WINDOW_WIDTH / 2 + 200, 100]),
        SFTunnel.Segment([WINDOW_WIDTH / 2 + 200, 100], [WINDOW_WIDTH
                         / 2 + 200, 200]),
        SFTunnel.Segment([0, 200], [WINDOW_WIDTH / 2 + 200, 200]),
        SFTunnel.Segment([WINDOW_WIDTH, 300], [WINDOW_WIDTH / 2 - 200,
                         300]),
        SFTunnel.Segment([WINDOW_WIDTH / 2 - 200, 300], [WINDOW_WIDTH
                         / 2 - 200, 400]),
        SFTunnel.Segment([WINDOW_WIDTH / 2 - 200, 400], [WINDOW_WIDTH,
                         400]),
        SFTunnel.Segment([0, 500], [WINDOW_WIDTH / 2 + 200, 500]),
        SFTunnel.Segment([WINDOW_WIDTH / 2 + 200, 500], [WINDOW_WIDTH
                         / 2 + 200, 600]),
        SFTunnel.Segment([0, 600], [WINDOW_WIDTH / 2 + 200, 600]),
        ]

    tunnel = SFTunnel(time_step=0.1, segments=segments)

    params = {
        'tunnel': tunnel,
        'position': [WINDOW_WIDTH / 2, 10],
        'destination': [WINDOW_WIDTH / 2, WINDOW_HEIGHT - 50],
        'radius': RADIUS,
        'assumed_speed': RADIUS * 1.3,
        'max_speed': RADIUS * 2.6,
        'velocity': [8, 8],
        }

    agent = SFAgent(**params)
    agents.append(agent)

    return (segments, agents)


def init_against(population, gap):

    agents = []
    tunnel = SFTunnel(time_step=0.1, segments=[])

    for i in range(int(population / 2)):

        pos = (400, 200 + gap * i)
        dest = (WINDOW_WIDTH - 400, 200 + gap * i)

        params = {
            'tunnel': tunnel,
            'position': pos,
            'destination': dest,
            'radius': RADIUS,
            'assumed_speed': RADIUS * 1.3,
            'max_speed': RADIUS * 2.6,
            'velocity': [8, 8],
            }

        agent = SFAgent(**params)
        agents.append(agent)

    for i in range(int(population / 2)):
        pos = (WINDOW_WIDTH - 400, 200 + gap * i)
        dest = (400, 200 + gap * i)

        params = {
            'tunnel': tunnel,
            'position': pos,
            'destination': dest,
            'radius': RADIUS,
            'assumed_speed': RADIUS * 1.3,
            'max_speed': RADIUS * 2.6,
            'velocity': [8, 8],
            }

        agent = SFAgent(**params)
        agents.append(agent)

    return agents


def init_circle(population):
    global RADIUS
    global WINDOW_HEIGHT
    global WINDOW_WIDTH

    r = min(WINDOW_WIDTH, WINDOW_HEIGHT) * 0.9 / 2
    incr = 2 * np.pi / population
    agents = []
    tunnel = SFTunnel(time_step=0.1, segments=[])

    for i in range(population):
        pos = (WINDOW_WIDTH / 2 + r * np.cos(incr * i), WINDOW_HEIGHT
               / 2 + r * np.sin(incr * i))
        dest = (WINDOW_WIDTH / 2 + r * np.cos(incr * i + np.pi),
                WINDOW_HEIGHT / 2 + r * np.sin(incr * i + np.pi))

        params = {
            'tunnel': tunnel,
            'position': pos,
            'destination': dest,
            'radius': RADIUS,
            'assumed_speed': RADIUS * 1.3,
            'max_speed': RADIUS * 2.6,
            'velocity': [8, 8],
            }

        agents.append(SFAgent(**params))
        agents[0].motility._assumed_speed = 14
        agents[0].motility._max_speed = 28

    return agents


def update_agents(agents):
    for agent in agents:
        agent.update()
        agent.move()


def f(
    s,
    e,
    t,
    duration,
    ):

    return int(s + int((e - s) * (t / float(duration))))


def draw_segments(segments):
    

    for segment in segments:
        (s1, e1) = (int(segment.start[0]), int(segment.start[1]))
        (s2, e2) = (int(segment.end[0]), int(segment.end[1]))
        pygame.draw.line(background, WHITE, (s1, e1), (s2, e2), 3)


def draw_agents(background, timespace, t):

    for position in timespace[t]:

        pygame.gfxdraw.aacircle(background, int(position[0]),
                                int(position[1]), int(10),
                                pygame.Color(250, 244, 100))
        pygame.gfxdraw.aacircle(background, int(position[0]),
                                int(position[1]), int(9),
                                pygame.Color(247, 237, 80))
        pygame.gfxdraw.filled_circle(background, int(position[0]),
                int(position[1]), int(8), BLACK)


def draw_paths(background, timespace, t):
    dur = PATH_LENGTH

    if t < dur:
        dur = t

    for i in range(dur):
        for position in timespace[t - i]:
            r = f(223, 1, i, dur)
            g = f(202, 1, i, dur)
            b = f(6, 1, i, dur)
            pygame.gfxdraw.aacircle(background, int(position[0]),
                                    int(position[1]), int(1),
                                    pygame.Color(r, g, b))


# --------------------------------------------------------------------------------------------
# Sim Setup

pygame.init()
fps_clock = pygame.time.Clock()
window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT),
                                 pygame.FULLSCREEN)
pygame.display.set_caption('SFTunev_pedestrian_dynamics_demo_v1.1')

background = pygame.Surface(window.get_size())
background = background.convert()
background.fill(BLACK)

window.blit(background, (0, 0))
pygame.display.flip()

segments = []
#(segments, agents) = init_bottleneck()
#agents = init_against(10, 80)
#agents = init_circle(9)
segments, agents = init_maze()

# sim_data = {}
# --------------------------------------------------------------------------------------------
# Main Loop

i = 0
timespace = {}
while True:
    if i > PATH_LENGTH:
        del timespace[i - PATH_LENGTH]

    positions = []
    for agent in agents:
        positions.append([int(agent.position[0]),
                         int(agent.position[1])])

    timespace[i] = positions
    background.fill(BLACK)
    update_agents(agents)

    draw_segments(segments)
    draw_paths(background, timespace, i)
    draw_agents(background, timespace, i)

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()

    window.blit(background, (0, 0))
    pygame.display.flip()
    fps_clock.tick(30)
    i += 1
