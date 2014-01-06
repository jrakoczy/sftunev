#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import sys
import utils
from datasources import FDSDataSource
from utils import Segment


class Agent(object):

    """ Generic class representing an evacuee. 
    """

    def __init__(self, **kwargs):
        """ Agent constructor. 

        Params: tunnel, position, simulation.
        """

        self._alive = True
        self._tunnel = kwargs['tunnel']
        self._position = np.array(kwargs['position'])
        self._simulation = kwargs['simulation']

    @property
    def alive(self):
        return self._alive

    @property
    def position(self):
        return self._position

    @property
    def tunnel(self):
        return self._tunnel

    @property
    def simulation(self):
        return self._simulation


class SFAgent(Agent):

    DEST_GAP = 1.5
    CHECKPOINT_GAP = 1.5
    CHECKPOINT_TOL = 1.5

    CO_MAX = 50000
    TEMP_MAX = 80

    FS_DIST_COEFF = 0.1

    def __init__(self, **kwargs):
        """ SFAgent constructor. 

        Finds the nearest checkpoint and the nearest exit.
        Params: tunnel, position, simulation, radius, assumed_speed,
        max_speed, velocity.
        """

        super().__init__(tunnel=kwargs['tunnel'],
                         position=kwargs['position'],
                         simulation=kwargs['simulation'])

        self._initial_position = self._position
        self._dest = None
        self._nearest_end = None
        self._checkpoint = None

        self.set_dest()
        self._find_nearest_end()
        self._find_checkpoint()

        self._time = 0.0
        self._reaction_time = 0.0
        self._calculate_reaction_time(kwargs['reaction_time'])
        self._radius = kwargs['radius']

        self._motivation = SFAgent.Motivation(self)

        # TODO motility and motiv in constr

        self._motility = SFAgent.Motility(agent=self,
                walk_speed=kwargs['walk_speed'],
                velocity=kwargs['velocity'],
                run_speed=kwargs['run_speed'])

    @property
    def initial_position(self):
        return self._initial_position

    @property
    def dest(self):
        return self._dest

    @property
    def checkpoint(self):
        return self._checkpoint

    @property
    def time(self):
        return self._time

    @property
    def radius(self):
        return self._radius

    @property
    def motility(self):
        return self._motility

    @property
    def motivation(self):
        return self._motivation

    def move(self):
        """
        Move of an agent with particular velocity in given time step.
        """

        if self._alive and self._reaction_time \
            < self._simulation.current_time:
            self._position = self.position + self.motility.velocity \
                * self._simulation.time_step

    def update(self):
        self._check_if_alive()

        if self._alive and self._reaction_time \
            < self._simulation.current_time:
            self._set_checkpoint()
            self.motility.update_velocity()
            self._time += self._simulation.time_step

    def _check_if_alive(self):
        current_time = self._simulation.current_time

        co_density = \
            self._tunnel.get_phys_at(FDSDataSource.CO_FNAME_KEY,
                current_time, self._position)
        temp = self._tunnel.get_phys_at(FDSDataSource.TEMP_FNAME_KEY,
                current_time, self._position)

        if co_density > self.CO_MAX or temp > self.TEMP_MAX:
            self._alive = False

    def _calculate_reaction_time(self, reaction_time):
        nearest_src = utils.find_nearest_segment(self._position,
                self._tunnel.fire_sources)
        src_mid = nearest_src.start + utils.vmiddle(nearest_src.end
                - nearest_src.start)
        dist = utils.vlen(src_mid - self._position)
        self._reaction_time = self.FS_DIST_COEFF * dist + reaction_time

    def set_dest(self):
        exits = self._tunnel.exits[:]
        dest = None

        while dest is None and len(exits) > 0:
            nearest_exit = utils.find_nearest_segment(self._position,
                    exits)

            exit_dest = nearest_exit.start \
                + utils.vmiddle(nearest_exit.end - nearest_exit.start)

            if self._check_exit_blockage(exit_dest):
                exits.remove(nearest_exit)
            else:
                dest = exit_dest

        if len(exits) == 0 and dest is None:
            nearest_exit = utils.find_nearest_segment(self._position,
                    self._tunnel.exits)

            dest = nearest_exit.start + utils.vmiddle(nearest_exit.end
                    - nearest_exit.start)

        dest_vec = utils.vnormalize(dest - self._position)
        self._dest = dest + dest_vec * self.DEST_GAP
        self._find_nearest_end()
        self._find_checkpoint()

    def _check_exit_blockage(self, exit_dest):
        dest_segment = Segment(self._position, exit_dest)

        for blockage in self._tunnel.temp_blockages:
            if utils.check_segments_intersect(blockage, dest_segment):
                return True

        return False

    def _set_checkpoint(self):
        if self._checkpoint is not None and utils.vlen(self._checkpoint
                - self.position) < self.CHECKPOINT_TOL \
            or self._checkpoint is None:

            self._find_nearest_end()
            self._find_checkpoint()

    def _find_checkpoint(self):
        if self._nearest_end is None:
            return None

        end_vector = self._nearest_end - self.position
        end_dir = utils.vnormalize(end_vector)
        self._checkpoint = self._nearest_end + self.CHECKPOINT_GAP \
            * end_dir

    def _find_nearest_end(self):

        dest_segment = Segment(self.position, self.dest)
        lowest_dist = sys.maxsize
        nearest_end = None

        if self._nearest_end is None:
            current_end = self._position
        else:
            current_end = self._nearest_end

        for segment in self.tunnel.segments:
            if utils.check_segments_intersect(dest_segment, segment):

                #   ! TODO ! - closest instead of nearest_end
                (dist, end) = utils.get_closer_end(self._position,
                        segment)

                if dist < lowest_dist and (end != current_end).all():
                    lowest_dist = dist
                    (_ , nearest_end)= utils.get_closer_end(self._dest, segment)

        self._nearest_end = nearest_end

    # ----------------------------------------------------
    # Motivation (inner class)
    # ----------------------------------------------------

    class Motivation:

        RELAXATION_TIME = 1  # [s]
        ANGLE_COEFF = 0.75
        DEST_FORCE_COEFF = 1
        SOCIAL_REP_COEFF = 3  # 1 (3)
        SOCIAL_DIST_COEFF = 0.5  # 40
        WALL_REP_COEFF = 5  #10  100 (50)
        WALL_DIST_REP = 1.5  # 450
        SOCIAL_DIST_TOL = 5
        WALL_DIST_TOL = 2.0
        WALL_REP_MAX = 1

        def __init__(self, agent):
            self._agent = agent

        def calculate_resultant(self):
            tunnel = self._agent.tunnel

            dest_force = self.DEST_FORCE_COEFF \
                * self._calculate_destination_force()
            social_rep = self.SOCIAL_REP_COEFF \
                * self._calculate_social_repulsion(tunnel.agents)
            wall_rep = self.WALL_REP_COEFF / self._agent.radius \
                * self._calculate_wall_repulsion(tunnel.segments)

            total_force = dest_force - social_rep - wall_rep

            return total_force

        # ----------------------------------------------------
        # destination component methods

        def _calculate_destination_force(self):
            """
            Calculates destination component of social force.
            """

            motility = self._agent.motility

            desired_speed = self._calculate_desired_speed()
            desired_direction = self._calculate_desired_direction()

            desired_velocity = desired_speed * desired_direction

            return (desired_velocity - motility.velocity) \
                / self.RELAXATION_TIME

        def _calculate_desired_direction(self):
            """
            """

            if self._agent.checkpoint is not None:
                destination = self._agent.checkpoint
            else:
                destination = self._agent.dest

            dest_vector = destination - self._agent.position
            return utils.vnormalize(dest_vector)

        def _calculate_desired_speed(self):
            """
            Calculates desired speed of pedestrian based on impatience factor.
            """

            motility = self._agent.motility

            impatience = self._calculate_impatience()
            desired_speed = (1 - impatience) * motility.assumed_speed \
                + impatience * motility.max_speed

            return desired_speed

        def _calculate_impatience(self):
            """
            Calculates impatience of a pedestrian taking into account average and initial speed ratio.
            """

            motility = self._agent.motility

            avg_speed = motility.calculate_avg_speed()
            impatience = 1 - avg_speed / motility.assumed_speed

            return impatience

        # ----------------------------------------------------
        # social repulsion methods

        def _calculate_social_repulsion(self, agents):
            total_force = np.array([0, 0])

            for agent in agents:
                if agent != self._agent and utils.vlen(agent.position
                        - self._agent.position) < self.SOCIAL_DIST_TOL:

                    angle_coeff = self._calculate_angle_coeff(agent)
                    rep_potential = \
                        self._calculate_soc_rep_potential(agent)
                    total_force = total_force + angle_coeff \
                        * rep_potential

            return total_force

        def _calculate_soc_rep_potential(self, agent):

            target_vector = agent.position - self._agent.position
            dist = utils.vlen(target_vector)
            target_dir = utils.vnormalize(target_vector)
            target_dir = self._rotate_direction_vetor(target_dir,
                    agent.position)

            return np.exp(agent.radius + self._agent.radius - dist
                          / self.SOCIAL_DIST_COEFF) * target_dir

        def _calculate_angle_coeff(self, agent):
            angle = self._calculate_target_angle(agent.position)
            return self._normalize_angle(angle)

        def _normalize_angle(self, angle):
            return self.ANGLE_COEFF + (1 - self.ANGLE_COEFF) * (1
                    + np.cos(angle)) / 2

        # ----------------------------------------------------
        # wall repulsion methods

        def _calculate_wall_repulsion(self, segments):
            total_force = np.array([0, 0])
            end_points = None

            for segment in segments:
                closest = \
                    utils.closest_on_segment(self._agent.position,
                        segment)

                if utils.vlen(closest - self._agent.position) \
                    > self.WALL_DIST_TOL:
                    continue

                rep_potential = np.array([0, 0])

                if end_points is None:
                    rep_potential = \
                        self._calculate_wall_rep_potential(closest)
                    end_points = np.array(closest)
                elif not utils.check_point_repetition(closest,
                        end_points):

                    rep_potential = \
                        self._calculate_wall_rep_potential(closest)
                    end_points = np.vstack([end_points, closest])

                total_force = total_force + rep_potential

            return total_force

        def _calculate_wall_rep_potential(self, closest):

            target_vector = closest - self._agent.position
            dist = utils.vlen(target_vector)
            target_dir = utils.vnormalize(target_vector)

            target_dir = self._rotate_direction_vetor(target_dir,
                    closest)

            return np.exp(-dist / self._agent.radius) * target_dir

        # ----------------------------------------------------
        # others

        def _calculate_target_angle(self, target):
            motility = self._agent.motility

            target_vector = target - self._agent.position
            return utils.vangle(motility.velocity, target_vector)

        def _rotate_direction_vetor(self, direction, target):
            angle = self._calculate_target_angle(target)

            # rot_angle = np.abs(np.cos(angle)) * np.pi / 4

            cos_angle = np.cos(angle)
            rot_angle = (cos_angle * np.pi / 4 if cos_angle > 0 else 0)
            return utils.vrotate(direction, rot_angle)

    # ----------------------------------------------------
    # Motility (inner class)
    # ----------------------------------------------------

    class Motility:

        MAX_SPEED_COEFF = 1.3

        def __init__(self, **kwargs):

            self._walk_speed = kwargs['walk_speed']
            self._assumed_speed = self._walk_speed
            self._max_speed = self.MAX_SPEED_COEFF * self._assumed_speed
            self._run_speed = kwargs['run_speed']
            self._velocity = np.array(kwargs['velocity'])
            self._agent = kwargs['agent']
            self._motivation = self._agent.motivation

        @property
        def assumed_speed(self):
            return self._assumed_speed

        @property
        def max_speed(self):
            return self._max_speed

        @property
        def velocity(self):
            return self._velocity

        def calculate_avg_speed(self):
            """
            Calculates average speed of pedestrian projecting their actual position on destination vector.
            """

            pos = self._agent.position - self._agent.initial_position
            dest = self._agent.dest - self._agent.initial_position
            proj_dist = utils.vproject(pos, dest)
            if self._agent.time == 0:
                avg_speed = utils.vlen(self.velocity)
            else:
                avg_speed = utils.vlen(proj_dist) / self._agent.time

            return avg_speed

        def update_velocity(self):
            simulation = self._agent.simulation
            motivation = self._agent.motivation
            tunnel = self._agent.tunnel
            temp = tunnel.get_phys_at(FDSDataSource.TEMP_FNAME_KEY,
                    simulation.current_time, self._agent.position)

            if temp > SFAgent.TEMP_MAX / 2:
                self._assumed_speed = self._run_speed
            else:
                self._assumed_speed = self._walk_speed

            self._max_speed = self.MAX_SPEED_COEFF * self._assumed_speed

            acc_vel_element = self._assumed_speed / self._walk_speed \
                * motivation.calculate_resultant() \
                * simulation.time_step
            self._velocity = self._velocity + acc_vel_element

    # ----------------------------------------------------
    # Cognition(inner class)
    # ----------------------------------------------------

    class Cognition:

        pass
