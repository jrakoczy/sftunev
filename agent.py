#!/usr/bin/python
# -*- coding: utf-8 -*-

from tunnel import SFTunnel
import numpy as np
import sys


class Agent(object):

    def __init__(self, **kwargs):

        self._tunnel = kwargs['tunnel']
        self._position = np.array(kwargs['position'])
        self._dest = np.array(kwargs['destination'])
        self._simulation = kwargs['_simulation']

    @property
    def position(self):
        return self._position

    @property
    def tunnel(self):
        return self._tunnel

    @property
    def dest(self):
        return self._dest


class SFAgent(Agent):

    TIME_STEP = 0.1  # [s]
    CHECKPOINT_GAP = 15
    CHECKPOINT_TOL = 30

    def __init__(self, **kwargs):
        super().__init__(tunnel=kwargs['tunnel'],
                         position=kwargs['position'],
                         destination=kwargs['destination'],
                         simulation=kwargs['simulation'])

        self._initial_position = self._position
        self._time = 0.0
        self._radius = kwargs['radius']

        self._motivation = SFAgent.Motivation(self)
        #TODO motility and motiv in constr
        self._motility = SFAgent.Motility(agent=self,
                assumed_speed=kwargs['assumed_speed'],
                max_speed=kwargs['max_speed'],
                velocity=kwargs['velocity'])

        self._nearest_end = None
        self._nearest_end = self._find_nearest_end()
        self._checkpoint = self._find_checkpoint(self._nearest_end)

    @property
    def initial_position(self):
        return self._initial_position

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

        new_position = self.position + self.motility.velocity \
            * self.tunnel.time_step
        self._position = new_position

    def update(self):
        self._set_checkpoint()
        self.motility.update_velocity()
        self._time += self.tunnel.time_step

    def _set_checkpoint(self):
        if self.checkpoint is not None and vlen(self.checkpoint
                - self.position) < self.CHECKPOINT_TOL:

            self._nearest_end = self._find_nearest_end()
            self._checkpoint = self._find_checkpoint(self._nearest_end)

    def _find_checkpoint(self, end):
        if end is None:
            return None

        end_vector = end - self.position
        end_dir = vnormalize(end_vector)
        return end + self.CHECKPOINT_GAP * end_dir

    def _find_nearest_end(self):

        dest_segment = SFTunnel.Segment(self.position, self.dest)
        lowest_dist = sys.maxsize
        nearest_end = None

        if self._nearest_end is None:
            current_end = self._position
        else:
            current_end = self._nearest_end

        for segment in self.tunnel.segments:
            if check_segments_intersect(dest_segment.start,
                    dest_segment.end, segment.start, segment.end):

                (dist, end) = get_closer_end(self.position,
                        segment.start, segment.end)

                if dist < lowest_dist and (end != current_end).all():
                    lowest_dist = dist
                    nearest_end = end

        return nearest_end

    # ----------------------------------------------------
    # Motivation (inner class)
    # ----------------------------------------------------

    class Motivation:

        RELAXATION_TIME = 1  # [s]
        ANGLE_COEFF = 0.75
        DEST_FORCE_COEFF = 5
        SOCIAL_REP_COEFF = 1  # 1
        SOCIAL_DIST_COEFF = 30  # 40
        WALL_REP_COEFF = 100  # 100
        WALL_DIST_REP = 350  # 450
        MAX_FORCE_VAL = 5  # 5

        SOCIAL_DIST_TOL = 50

        def __init__(self, agent):
            self._agent = agent

        def calculate_resultant(self):
            tunnel = self._agent.tunnel

            dest_force = self.DEST_FORCE_COEFF \
                * self._calculate_destination_force()
            social_rep = self.SOCIAL_REP_COEFF \
                * self._calculate_social_repulsion(tunnel.agents)
            wall_rep = self.WALL_REP_COEFF \
                * self._calculate_wall_repulsion(tunnel.segments)

            total_force = dest_force - social_rep - wall_rep

            if vlen(total_force) > self.MAX_FORCE_VAL:
                return self.MAX_FORCE_VAL * vnormalize(total_force)
            else:
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
            return vnormalize(dest_vector)

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
                if agent != self and vlen(agent.position
                        - self._agent.position) < self.SOCIAL_DIST_TOL:
                    angle_coeff = self._calculate_angle_coeff(agent)
                    rep_potential = \
                        self._calculate_soc_rep_potential(agent)
                    total_force = total_force + angle_coeff \
                        * rep_potential

            return total_force

        def _calculate_soc_rep_potential(self, agent):

            target_vector = agent.position - self._agent.position
            dist = vlen(target_vector)
            target_dir = vnormalize(target_vector)
            target_dir = self._rotate_direction_vetor(target_dir,
                    agent.position)

            return np.exp(self.SOCIAL_DIST_COEFF + agent.radius
                          + self._agent.radius - dist) * target_dir

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

            for segment in segments:
                rep_potential = \
                    self._calculate_wall_rep_potential(segment)
                total_force = total_force + rep_potential

            return total_force

        def _calculate_wall_rep_potential(self, segment):
            closest = closest_on_segment(self._agent.position,
                    segment.start, segment.end)
            target_vector = closest - self._agent.position
            dist = vlen(target_vector)
            target_dir = vnormalize(target_vector)

            # target_dir = self._rotate_direction_vetor(target_dir,
             #       closest)

            return np.exp(-dist + self.WALL_DIST_REP
                          / self._agent.radius) * target_dir

        # ----------------------------------------------------
        # others

        def _calculate_target_angle(self, target):
            motility = self._agent.motility

            target_vector = target - self._agent.position
            return vangle(motility.velocity, target_vector)

        def _rotate_direction_vetor(self, direction, target):
            angle = self._calculate_target_angle(target)

            # rot_angle = np.abs(np.cos(angle)) * np.pi / 4

            cos_angle = np.cos(angle)
            rot_angle = (cos_angle * np.pi / 4 if cos_angle > 0 else 0)
            return vrotate(direction, rot_angle)

    # ----------------------------------------------------
    # Motility (inner class)
    # ----------------------------------------------------

    class Motility:

        def __init__(self, **kwargs):

            self._assumed_speed = kwargs['assumed_speed']
            self._max_speed = kwargs['max_speed']
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
            proj_dist = vproject(pos, dest)
            if self._agent.time == 0:
                avg_speed = vlen(self.velocity)
            else:
                avg_speed = vlen(proj_dist) / self._agent.time

            return avg_speed

        def update_velocity(self):
            tunnel = self._agent.tunnel
            motivation = self._agent.motivation

            acc_vel_element = motivation.calculate_resultant() \
                * tunnel.time_step
            self._velocity = self._velocity + acc_vel_element

    # ----------------------------------------------------
    # Cognition(inner class)
    # ----------------------------------------------------

    class Cognition:

        pass


# ----------------------------------------------------

def vlensq(v):
    v = np.array(v)
    return np.vdot(v, v)


def vlen(v):
    v = np.array(v)
    return np.sqrt(vlensq(v))


def vnormalize(v):
    v = np.array(v)
    length = vlen(v)
    if length != 0:
        return v / length
    else:
        return v


def vproject(v1, v2):
    """ project a onto b

    formula: b(dot(a,b)/(|b|^2))
    """

    v1 = np.array(v1)
    v2 = np.array(v2)

    if (v1 != v2).any():
        dot_product = np.vdot(v1, v2)
        lensq = vlensq(v2)
        temp = float(dot_product) / float(lensq)
        c = temp * v2
    else:
        c = np.array([0, 0])  # TODO

    return c


def vangle(v1, v2):
    u1 = vnormalize(v1)
    u2 = vnormalize(v2)
    dot_product = np.vdot(u1, u2)
    angle = np.arccos(dot_product)

    if np.isnan(angle):
        if (u1 == u2).all():
            return 0.0
        else:
            return np.pi

    return angle


def vrotate(v, angle):

    a_cos = np.cos(angle)
    a_sin = np.sin(angle)
    x = v[0] * a_cos - v[1] * a_sin
    y = v[1] * a_cos + v[0] * a_sin

    return np.array([x, y])


def closest_on_segment(p, start, end):

    p = np.array(p)
    start = np.array(start)
    end = np.array(end)

    # segment length

    seg_len = vlen(end - start)

    # in case the segment points are equal

    if seg_len < 0.00000001:
        return end

    u1 = (p[0] - start[0]) * (end[0] - start[0]) + (p[1] - start[1]) \
        * (end[1] - start[1])
    u = u1 / (seg_len * seg_len)

    # in case that one of end points is the closest

    if u < 0.00001 or u > 1:
        dist_to_start = vlen(start - p)
        dist_to_end = vlen(end - p)

        if dist_to_start > dist_to_end:
            closest = end
        else:
            closest = start
    else:

    # closest point lays between start and end

        ix = start[0] + u * (end[0] - start[0])
        iy = start[1] + u * (end[1] - start[1])
        closest = np.array([ix, iy])

    return closest


def check_points_ccw(px, py, pz):
    px = np.array(px)
    py = np.array(py)
    pz = np.array(pz)

    return (pz[1] - px[1]) * (py[0] - px[0]) > (py[1] - px[1]) * (pz[0]
            - px[0])


def check_segments_intersect(
    start1,
    end1,
    start2,
    end2,
    ):

    return check_points_ccw(start1, start2, end2) \
        != check_points_ccw(end1, start1, end2) \
        and check_points_ccw(start1, end1, start2) \
        != check_points_ccw(start1, end1, end2)


def get_closer_end(p, start, end):
    start_dist = vlen(start - p)
    end_dist = vlen(end - p)

    if start_dist < end_dist:
        return (start_dist, start)
    else:
        return (end_dist, end)
