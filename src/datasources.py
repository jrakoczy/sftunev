#!/usr/bin/python
# -*- coding: utf-8 -*-

import re
from os import listdir
from os.path import join, isfile
import numpy as np
from sys import stdout
from utils import Segment
from utils import show_loading_progress, root_out_none_values


class GenericDataSource(object):

    """Generic class representing various data sources.

    Method get_phys_conditions_at should be overriden.
    """

    FDS_DATA_NAME = 'FDS'

    @property
    def sim_duration(self):
        """Simulation time getter.

        Returns:
        total simulation duration [s].
        """

        if self._sim_duration is not None:
            return self._sim_duration
        else:
            raise NoDataError('Simulation time not defined')

    @property
    def tunnel_dimensions(self):
        """Tunnel dimensions getter. 

        Returns:
        dictionary: height, width, length, resolution [m].
        """

        if self._tunnel_dimensions is not None:
            return self._tunnel_dimensions
        else:
            raise NoDataError('Tunnel dimensions not defined')

    @property
    def exits(self):
        """Exits getter.

        Returns: list of exits represented by segments.
        """

        if self._exits is not None:
            return self._exits
        else:
            raise NoDataError('Exits not defined')

    @property
    def fire_sources(self):
        """Fire sources getter.

        Returns: list of fire sources represented by points.
        """

        if self._fire_sources is not None:
            return self._fire_sources
        else:
            raise NoDataError('Fire sources not defined')

    @property
    def phys_conditions(self):
        if self._phys_conditions is not None:
            return self._phys_conditions
        else:
            return NoDataError('Physical conditions not defined')

    @property
    def obstacles(self):
        """ Obstacles getter.

        Returns: list of obstacles represented by their diagonals.
        """

        if self._obstacles is not None:
            return self._obstacles
        else:
            return NoDataError('Obstacles not defined')

    def factory(type, dir_path=None):
        if type == GenericDataSource.FDS_DATA_NAME:
            return FDSDataSource(dir_path)
        else:
            return None


class FDSDataSource(GenericDataSource):

    """Class representing data generated by FDS.
    """

    # constants

    (FDS_FNAME_KEY, TEMP_FNAME_KEY, CO_FNAME_KEY) = ('fds', 'temp', 'co'
            )

    # regular expressions

    FDS_VAL_REGEXP = '[0-9]+\\.?[0-9]?'
    FDS_EXP_VAL_REGEXP = '[-+]?[0-9]*\.?[0-9]+[eE][-+]?[0-9]+'
    GROUPED_VAL_REGEXP = '\\s*(' + FDS_VAL_REGEXP + ')\\s*'
    NON_GROUPED_VAL_REGEXP = '\\s*' + FDS_VAL_REGEXP + '\\s*'

    DIMENSIONS_REGEXP = re.compile('^&MESH\\s+IJK\\s*='
                                   + GROUPED_VAL_REGEXP + ','
                                   + GROUPED_VAL_REGEXP + ','
                                   + GROUPED_VAL_REGEXP + ',\\s*XB\\s*='
                                    + GROUPED_VAL_REGEXP + ','
                                   + GROUPED_VAL_REGEXP + ','
                                   + GROUPED_VAL_REGEXP + ','
                                   + GROUPED_VAL_REGEXP + ','
                                   + GROUPED_VAL_REGEXP + ','
                                   + GROUPED_VAL_REGEXP)

    F_SRC_REGEXP = re.compile('^&OBST\\s+XB\\s*=' + GROUPED_VAL_REGEXP
                              + ',\\s*' + GROUPED_VAL_REGEXP + ',\\s*'
                              + GROUPED_VAL_REGEXP + ',\\s*'
                              + GROUPED_VAL_REGEXP + ',\\s*'
                              + GROUPED_VAL_REGEXP + ',\\s*'
                              + GROUPED_VAL_REGEXP
                              + ",\\s*SURF_ID='fire'")

    EXITS_REGEXP = re.compile('^&HOLE\\s+XB\\s*=' + GROUPED_VAL_REGEXP
                              + ',' + GROUPED_VAL_REGEXP + ','
                              + GROUPED_VAL_REGEXP + ','
                              + GROUPED_VAL_REGEXP + ','
                              + NON_GROUPED_VAL_REGEXP + ','
                              + NON_GROUPED_VAL_REGEXP)

    OBST_REGEXP = re.compile('^&OBST\\s+XB\\s*=' + GROUPED_VAL_REGEXP
                             + ',\\s*' + GROUPED_VAL_REGEXP + ',\\s*'
                             + GROUPED_VAL_REGEXP + ',\\s*'
                             + GROUPED_VAL_REGEXP + ',\\s*'
                             + GROUPED_VAL_REGEXP + ',\\s*'
                             + GROUPED_VAL_REGEXP + ",\\s*SURF_ID='CAR'"
                             )

    DURATION_REGEXP = re.compile('^&TIME\\s+T_END='
                                 + GROUPED_VAL_REGEXP)

    FNAME_REGEXP = \
        {TEMP_FNAME_KEY: re.compile('^temp_(\\d+)-(\\d+)\\.csv'),
         CO_FNAME_KEY: re.compile('^co_(\\d+)-(\\d+)\\.csv'),
         FDS_FNAME_KEY: re.compile('^.*\\.fds$')}

    PHYS_FNAME_REGEXP = re.compile('^\w+_(\\d+)-(\\d+)\\.csv')
    PHYS_MEASURES_REGEXP = re.compile('^\\s*(' + FDS_EXP_VAL_REGEXP
            + '),\\s*(' + FDS_EXP_VAL_REGEXP + '),\\s*('
            + FDS_EXP_VAL_REGEXP + ')\\s*')

    # methods

    def __init__(self, dir_path):
        """ FDSDataSource constructor 

        Creates an instance of FDSDataSource. 
        Data is retrieved from files stored in dir_path.
        Automatically issues parse_data().
        """

        self._dir_path = dir_path
        self._filenames = dict()
        self._categorize_data_files()

        self._sim_duration = None
        self._tunnel_dimensions = None
        self._exits = None
        self._fire_sources = None
        self._phys_conditions = None
        self._interval_length = None
        self.parse_data()

    def parse_data(self):
        """ Parses FDS data.

        Retrieves data from FDS configuration and output files.
        """

        self._parse_fds_file()
        self._load_phys_conditions_data()

    @property
    def filenames(self):
        return self._filenames

    @property
    def interval_length(self):
        return self._interval_length

    def _categorize_data_files(self):
        """ Categorizes data files located in dir_path.

        Creates a dictionary using file names as keys. 
        """

        for k in FDSDataSource.FNAME_REGEXP.keys():
            self._filenames[k] = \
                extract_files_by_regexp(self._dir_path,
                    FDSDataSource.FNAME_REGEXP[k])

    # .fds file

    def _parse_fds_file(self):
        """ Parses .fds file. 

        Finds and stores passageway dimensions, exits, obstacles 
        and total duration of simulation.
        """

        self._find_dimensions()
        self._find_fire_sources()
        self._find_exits()
        self._find_obstacles()
        self._find_duration()

    def _find_dimensions(self):
        """ Finds passageway dimensions. 
 
        Creates a dictionary: height, width, length, grid resolution.
        """

        filepath = self._get_fds_file_path()
        fds_values = extract_line_by_regexp(filepath,
                FDSDataSource.DIMENSIONS_REGEXP)

        if len(fds_values) != 1:
            raise IncorrectDataError('FDS file formatted improperly')  # multidimensional list indicates that there was more than

                                                                       # one declaration of model dimensions in .fds file

        if fds_values[0][0] == 0:
            raise IncorrectDataError('Cannot calculate grid resolution.'
                    )

        self._tunnel_dimensions = {
            'height': float(fds_values[0][8]),
            'width': float(fds_values[0][4]),
            'length': float(fds_values[0][6]),
            'resolution': float(fds_values[0][4]) \
                / float(fds_values[0][0]),
            }

        self._check_dimensions()

    def _check_dimensions(self):
        """ Checks corectness of dimensions values. 

        Raises an exception if any of values is negative.
        """

        for v in self.tunnel_dimensions.values():
            if v <= 0:
                raise IncorrectDataError('Model impossible to render. Incorrect dimensions.'
                        )

    def _find_fire_sources(self):
        """ Finds fire sources. 

        Creates list of fire sources represented by their diagonals.
        """

        filepath = self._get_fds_file_path()
        fds_values = extract_line_by_regexp(filepath,
                FDSDataSource.F_SRC_REGEXP)
        self._fire_sources = []

        for coords in fds_values:
            start = np.array([float(coords[0]), float(coords[2])])
            end = np.array([float(coords[1]), float(coords[3])])
            self._fire_sources.append(Segment(start, end))

    def _find_exits(self):
        """ Finds tunnel exits. 

        Creates list of exits represented by segments.
        """

        filepath = self._get_fds_file_path()
        fds_values = extract_line_by_regexp(filepath,
                FDSDataSource.EXITS_REGEXP)
        self._exits = []

        for coords in fds_values:
            start = [float(coords[0]), float(coords[2])]
            end = [float(coords[1]), float(coords[3])]
            self._exits.append(Segment(start, end))

        self._find_openings()

    def _find_openings(self):
        """ Finds openings at both passageway ends. 
        
        Appends found segments to the exits list.
        """

        width = self._tunnel_dimensions['width']
        length = self._tunnel_dimensions['length']

        self._exits.append(Segment([0, 0], [width, 0]))
        self._exits.append(Segment([0, length], [width, length]))

    def _find_obstacles(self):
        """ Finds rectangular obstacles.

        Creates list of obstacles represented by their diagonals.
        """

        filepath = self._get_fds_file_path()
        fds_values = extract_line_by_regexp(filepath,
                FDSDataSource.OBST_REGEXP)
        self._obstacles = []

        for coords in fds_values:
            start = [float(coords[0]), float(coords[2])]
            end = [float(coords[1]), float(coords[3])]
            self._obstacles.append(Segment(start, end))

    def _find_duration(self):
        """ Finds simulation duration. 
        """

        filepath = self._get_fds_file_path()
        fds_values = extract_line_by_regexp(filepath,
                FDSDataSource.DURATION_REGEXP)
        duration = float(fds_values[0][0])

        if duration < 0:
            raise IncorrectDataError('Negative time.')

        self._sim_duration = duration

    def _get_fds_file_path(self):
        """ Returns .fds file path.
        """

        return join(self._dir_path,
                    self.filenames[FDSDataSource.FDS_FNAME_KEY][0])  # there shouldn't be more than 1 .fds file

    # physical condition files

    def _load_phys_conditions_data(self):
        """ Loads physical conditions data. 
        
        Creates data concerning CO density and temperature. 
        """

        self._phys_conditions = {FDSDataSource.TEMP_FNAME_KEY: {},
                                 FDSDataSource.CO_FNAME_KEY: {}}
        self._load_physical_measures(FDSDataSource.TEMP_FNAME_KEY,
                show_loading_progress)
        self._load_physical_measures(FDSDataSource.CO_FNAME_KEY,
                show_loading_progress)
        self._interval_length = \
            self._get_interval_length(self._filenames[FDSDataSource.TEMP_FNAME_KEY][0])

    def _load_physical_measures(self, phys_cond_key, status_display):
        """ Loads physical measures defined by phys_cond_key. 

        Creates values of phys_cond_key for given time intervals. 
        Each parsed file contains data for different interval.
        """

        files_total = len(self.filenames[phys_cond_key])
        current = 0

        for fname in self.filenames[phys_cond_key]:
            filepath = join(self._dir_path, fname)
            initial_time = self._get_initial_time(fname)
            measures = self._exctract_raw_measures(filepath)

            self._phys_conditions[phys_cond_key][initial_time] = \
                measures

            current += 1
            status_display(current, files_total, phys_cond_key
                           + ' files scanned: ')

        stdout.write('\n')

    def _exctract_raw_measures(self, filepath):
        """ Extracts values from a single physical measures data file. 

        Returns: dictionary of measure values at a particular position.
        """

        measures = {}

        with open(filepath, 'r') as f:
            for line in f.readlines():
                matcher = FDSDataSource.PHYS_MEASURES_REGEXP.match(line)

                if matcher is not None:
                    point = (float(matcher.group(1)),
                             float(matcher.group(2)))
                    value = float(matcher.group(3))
                    measures[point] = value

        return measures

    def _get_initial_time(self, filename):
        """ Extracts an initial time of an interval.
        """

        matcher = FDSDataSource.PHYS_FNAME_REGEXP.match(filename)
        return float(matcher.group(1))

    def _get_interval_length(self, filename):
        """ Calculates time step length between following measures.
        """
        matcher = FDSDataSource.PHYS_FNAME_REGEXP.match(filename)
        return float(matcher.group(2)) - float(matcher.group(1))


def extract_files_by_regexp(dir_path, regexp):
    """Finds a file name matching defined regexp.

    Searches for results in dir_path. 
    Returns: regular files only. Omits directories.
    """

    filenames = []
    for fname in listdir(dir_path):
        filepath = join(dir_path, fname)
        if isfile(filepath) and regexp.match(fname) is not None:
            filenames.append(fname)

    return filenames


def extract_line_by_regexp(file_path, regexp):
    """ Extracts values from file_path line by line. 

    Finds values matching defined regexp.
    Returns: list of extracted values.
    """

    matched_data = []
    with open(file_path, 'r') as f:

        for line in f.readlines():
            matcher = regexp.match(line)
            if matcher is not None:
                extracted_values = \
                    root_out_none_values(matcher.groups())
                matched_data.append(extracted_values)

    if not matched_data:
        raise NoDataError('Data not found')

    return matched_data


class NoDataError(Exception):

    pass


class IncorrectDataError(Exception):

    pass

