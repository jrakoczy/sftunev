#!/usr/bin/python
# -*- coding: utf-8 -*-


class Simulation(object):

    def __init__(self, time_step):
        self._time_step = time_step

    @property
    def time_step(self):
        return self._time_step
