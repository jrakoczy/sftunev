#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np


class Tunnel(object):

    #tests
    def __init__(self, time_step):
        self._time_step = time_step


    @property
    def time_step(self):
        return self._time_step


class SFTunnel(Tunnel):

    #tests
    def __init__(
        self,
        time_step,
        agents=[],
        segments=[],
        ):
        super().__init__(time_step=time_step)

        self._agents = agents
        self._segments = segments

    def __init__(self, data_source, agents):
        super().__init__()

    @property
    def agents(self):
        return self._agents

    @property
    def segments(self):
        return self._segments

    def add_agent(self, agent):
        self._agents.append(agent)

    class Segment:

        def __init__(self, start, end):
            self._start = np.array(start)
            self._end = np.array(end)

        @property
        def start(self):
            return self._start

        @property
        def end(self):
            return self._end
