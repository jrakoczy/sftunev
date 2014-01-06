#!/usr/bin/python
# -*- coding: utf-8 -*-

from datasources import GenericDataSource
from tunnel import SFTunnel
import numpy as np
import random
from agent import SFAgent
import utils
import time


class Simulation(object):

    def __init__(self, time_step):
        self._time_step = time_step
        self._duraton = None
        self._current_time = 0

    @property
    def time_step(self):
        return self._time_step

    @property
    def current_time(self):
        return self._current_time


class SFSimulation(Simulation):

    OBST_DIST = 1
    RADIUS = 0.3
    DEST_TOL = 2.5

    def __init__(self, time_step, dir_path):
        super().__init__(time_step)

        self._data_source = GenericDataSource.factory('FDS', dir_path)
        self._agents = []
        self._rescued = []
        self._filenames = {}
        self._tunnel = SFTunnel(self._data_source, self)
        self._spawn_agents()

    @property
    def agents(self):
        return self._agents

    @property
    def tunnel(self):
        return self._tunnel

    @property
    def time_step(self):
        return self._time_step

    def simulate(self, status_display):

        duration = self._data_source.sim_duration + self._time_step

        interval_length = self._data_source.interval_length

        for t in np.arange(0, duration, self._time_step):
            status_display(t, duration, 'simulating: ')
            self._save_agent_states()
            self._current_time = t

            if t != 0 and t % interval_length == 0:
                self._update_temp_blockages()

            self._remove_rescued()
            self._update_agents()
            self._move_agents()


    def _remove_rescued(self):
        rescued = []

        for agent in self._agents:
            if utils.vlen(agent.position - agent.dest) < self.DEST_TOL:
                rescued.append(agent)

        for rescued_agent in rescued:
            self._agents.remove(rescued_agent)

        self._rescued.extend(rescued)

    def _update_agents(self):
        for agent in self._agents:
            if agent.alive:
                agent.update()

    def _move_agents(self):
        for agent in self._agents:
            if agent.alive:
                agent.move()

    def _save_agent_states(self):
        for agent in self._agents:
            condition = 'DEAD'
            if agent.alive:
                condition = 'STANDING'

            self._state_to_file(agent, condition)

        for agent in self._rescued:
            condition = 'HIDDEN'
            self._state_to_file(agent, condition)

    def _state_to_file(self, agent, condition):
        pos = '[ %.5f %.5f]' % (agent.position[0], agent.position[1])

        with open(self._filenames[agent], 'a') as f:
            f.write(str(self._current_time) + '|' + condition + '|'
                    + str(pos) + '\n')

    def _update_temp_blockages(self):
        self._tunnel.find_temp_blockage()
        for agent in self._agents:
            agent.set_dest()

    def _spawn_agents(self):
        obstacles = self._data_source.obstacles
        (walk_speed, run_speed) = self._generate_velocity()
        reaction_time = random.gauss(2, 0.5)

        for obstacle in obstacles:
            init_position = np.array([obstacle.end[0]
                    + SFSimulation.OBST_DIST, obstacle.end[1]])
            kwargs = {
                'simulation': self,
                'tunnel': self._tunnel,
                'position': init_position,
                'radius': SFSimulation.RADIUS,
                'walk_speed': walk_speed,
                'run_speed': run_speed,
                'velocity': [0, 0],
                'reaction_time': reaction_time,
                }

            agent = SFAgent(**kwargs)
            self._agents.append(agent)
            self._generate_filenames()

    def _generate_velocity(self):

        if random.random() < 0.1:
            mov_speed = random.gauss(0.8, 0.15)
            run_speed = mov_speed
        else:
            mov_speed = random.gauss(1.3, 0.3)
            run_speed = random.gauss(4.1, 0.5)

        return (mov_speed, run_speed)

    def _generate_filenames(self):
        i = 0
        for agent in self._agents:
            self._filenames[agent] = 'results/evacuee' + str(i) + '.tem'
            i += 1


if __name__ == '__main__':
    sim = SFSimulation(0.1, 'dane') 
     
    start_time = time.time()
    sim.simulate(utils.show_loading_progress)
    print(time.time() - start_time)