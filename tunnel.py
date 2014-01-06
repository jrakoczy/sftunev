#!/usr/bin/python
# -*- coding: utf-8 -*-

from datasources import GenericDataSource
import utils
from datasources import FDSDataSource
from agent import SFAgent
from utils import Segment
import numpy as np


class Tunnel(object):

    def __init__(self, data_source):
        self._data_source = data_source


class SFTunnel(Tunnel):

    def __init__(self, data_source, simulation):
        super().__init__(data_source)
        self._simulation = simulation
        self._agents = []
        self._segments = []
        self._temp_blockages = []
        self._create_wall_segments()
        self._create_obstacles()


    @property
    def dimensions(self):
        return self._data_source.tunnel_dimensions

    @property
    def agents(self):
        return self._agents

    @property
    def segments(self):
        return self._segments

    @property
    def exits(self):
        return self._data_source.exits

    @property
    def fire_sources(self):
        return self._data_source.fire_sources

    @property
    def temp_blockages(self):
        return self._temp_blockages

    def add_agent(self, agent):
        self._agents.append(agent)

    def get_phys_at(
        self,
        phys_cond_key,
        t,
        pos,
        ):
        res = self._data_source.tunnel_dimensions['resolution']
        pos = utils.around_to_precision(pos, res)
        pos = (pos[0], pos[1])  # numpy.array is mutable == unhashable

        interval_length = self._data_source.interval_length
        t = utils.around_to_precision(t, interval_length)
        
        try:
            value = self._data_source.phys_conditions[phys_cond_key][t][pos]
        except KeyError:
            value = 0

        return value

    # ----------------------------------------------------
    # temperature blockage

    def find_temp_blockage(self):
        self._temp_blockages = []

        res = self._data_source.tunnel_dimensions['resolution']
        length = self._data_source.tunnel_dimensions['length']
        width = self._data_source.tunnel_dimensions['width']
        current_time = self._simulation.current_time

        for y in np.arange(0, length, res):
            pos = np.array([0, y])
            temp = self.get_phys_at(FDSDataSource.TEMP_FNAME_KEY,
                                    current_time, pos)
            block_flag = False

            if temp > SFAgent.TEMP_MAX:
                block_flag = True

                for x in np.arange(res, width, res):
                    pos = np.array([x, y])
                    temp = \
                        self.get_phys_at(FDSDataSource.TEMP_FNAME_KEY,
                            current_time, pos)

                    if temp < SFAgent.TEMP_MAX/3:
                        block_flag = False
                        break
            
            if block_flag:
                self._temp_blockages.append(Segment([0, y], [width, y]))

    # ----------------------------------------------------
    # wall semgents

    def _create_wall_segments(self):
        exits = self._data_source.exits
        (wall1, wall2) = self._create_walls()
        wall1_segments = [wall1]
        wall2_segments = [wall2]

        for exit in exits:
            if self._is_exit_on_wall(exit, wall1):
                wall1_segments = self._cut_out_exit(exit,
                        wall1_segments)

            if self._is_exit_on_wall(exit, wall2):
                wall2_segments = self._cut_out_exit(exit,
                        wall2_segments)

        self._segments.extend(wall1_segments)
        self._segments.extend(wall2_segments)

    def _is_exit_on_wall(self, exit, segment):
        return exit.start[0] == segment.start[0] and exit.end[0] \
            == segment.end[0]

    def _is_exit_on_segment(self, exit, segment):
        return exit.start[1] > segment.start[1] and exit.end[1] \
            < segment.end[1]

    def _cut_out_exit(self, exit, segments):
        for segment in segments:
            if self._is_exit_on_segment(exit, segment):
                new_segments = self._split_segment(exit, segment)
                segments.remove(segment)
                segments.extend(new_segments)
                return segments

        raise InproperCoordsError

    def _split_segment(self, exit, segment):
        new_segment1 = Segment(segment.start, exit.start)
        new_segment2 = Segment(exit.end, segment.end)

        return [new_segment1, new_segment2]

    def _create_walls(self):
        width = self._data_source.tunnel_dimensions['width']
        length = self._data_source.tunnel_dimensions['length']
        wall1 = Segment([0, 0], [0, length])
        wall2 = Segment([width, 0], [width, length])

        return (wall1, wall2)

    # ----------------------------------------------------
    # obstacles

    def _create_obstacles(self):
        obstacles = self._data_source.obstacles
        segments = []

        for obstacle in obstacles:
            segment1 = Segment(obstacle.start, [obstacle.start[0],
                               obstacle.end[1]])
            segement2 = Segment([obstacle.start[0], obstacle.end[1]],
                                obstacle.end)
            segment3 = Segment([obstacle.end[0], obstacle.start[1]],
                               obstacle.end)
            segment4 = Segment(obstacle.start, [obstacle.end[0],
                               obstacle.start[1]])

            segments.extend([segment1, segement2, segment3, segment4])

        self._segments.extend(segments)


class InproperCoordsError(Exception):

    pass


if __name__ == '__main__':
    src = GenericDataSource.factory('FDS', 'dane')
    tunnel = SFTunnel(src)
    for segment in tunnel.segments:
        print (segment.start, segment.end)
