#!/usr/bin/python
# -*- coding: utf-8 -*-

import re
from os import listdir
from os.path import join, isfile


class GenericDataSource(object):

    """Generic class for various data sources.

    Method get_phys_conditions_at should be overriden.
    """
    DATA_SOURCE_TYPES = ('FDS',)

    @property
    def sim_duration(self):
        """Simulation time getter.

    ....Returns total simulation duration [s].
    ...."""

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

        Returns: list of exits. Exits are tuples consitiong of 2 dicionaries representing points.
    ...."""

        if self._exits is not None:
            return self._exits
        else:
            raise NoDataError('Exits not defined')

    @property
    def fire_sources(self):
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

    def get_phys_conditions_at(self, position):
        raise NotImplementedError('Should have implemented this')

    def get_model_data(self):
        """Returns basic model data.

        Creates a dictionary containing sim_duration, tunnel_dimensions,
        exits, fire_sources
        """

        return {
            'sim_duration': self.sim_duration,
            'tunnel_dimensions': self.tunnel_dimensions,
            'exits': self.exits,
            #'fire_sources': self.fire_sources,
            }

    def factory(type, dir_path = None):
    	if type == 'FDS':
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

    EXITS_REGEXP = re.compile('^&HOLE\\s+XB\\s*=' + GROUPED_VAL_REGEXP
                              + ',' + GROUPED_VAL_REGEXP + ','
                              + GROUPED_VAL_REGEXP + ','
                              + GROUPED_VAL_REGEXP + ','
                              + NON_GROUPED_VAL_REGEXP + ','
                              + NON_GROUPED_VAL_REGEXP)

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
        self._dir_path = dir_path
        self._filenames = dict()
        self._categorize_data_files()

        self._sim_duration = None
        self._tunnel_dimensions = None
        self._exits = None
        self._fire_sources = None
        self._phys_conditions = None

    def parse_data(self):
    	self._parse_fds_file()
    	self._load_phys_conditions_data()

    def _categorize_data_files(self):
        for k in FDSDataSource.FNAME_REGEXP.keys():
            self._filenames[k] = \
                extract_files_by_regexp(self._dir_path,
                    FDSDataSource.FNAME_REGEXP[k])


    # .fds file

    def _parse_fds_file(self):
        self._find_dimensions()
        self._find_exits()
        self._find_duration()

    def _find_dimensions(self):
        filepath = self._get_fds_file_path()
        fds_values = extract_line_by_regexp(filepath,
                FDSDataSource.DIMENSIONS_REGEXP)

        if len(fds_values) != 1:
            raise IncorrectDataError('FDS file formatted improperly')  # multidimensional list indicates that there was more than

                                                                       # one declaration of model dimensionsin .fds file

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
        for v in self.tunnel_dimensions.values():
            if v <= 0:
                raise IncorrectDataError('Model impossible to render with declared dimensions.'
                        )

    def _find_exits(self):
        filepath = self._get_fds_file_path()
        fds_values = extract_line_by_regexp(filepath,
                FDSDataSource.EXITS_REGEXP)
        self._exits = []

        for coords in fds_values:
            point1 = {'x': coords[0], 'y': coords[2]}
            point2 = {'x': coords[1], 'y': coords[3]}
            self._exits.append((point1, point2))

    def _find_duration(self):
        filepath = self._get_fds_file_path()
        fds_values = extract_line_by_regexp(filepath,
                FDSDataSource.DURATION_REGEXP)
        duration = float(fds_values[0][0])

        if duration < 0:
            raise IncorrectDataError('Negative time.')

        self._sim_duration = duration

    def _get_fds_file_path(self):
        return join(self._dir_path,
                    self.filenames[FDSDataSource.FDS_FNAME_KEY][0])  # there shouldn't be more than 1 .fds file

    # physical condition files

    def _load_phys_conditions_data(self):
        self._phys_conditions = {FDSDataSource.TEMP_FNAME_KEY: {},
                                 FDSDataSource.CO_FNAME_KEY: {}}
        self._load_physical_measures(FDSDataSource.TEMP_FNAME_KEY, show_loading_progress)
        self._load_physical_measures(FDSDataSource.CO_FNAME_KEY, show_loading_progress)

    def _load_physical_measures(self, phys_cond_key, status_display):
        files_total = len(self.filenames[phys_cond_key])
        current = 0

        for fname in self.filenames[phys_cond_key]:
            filepath = join(self._dir_path, fname)
            initial_time = self._get_initial_time(fname)
            measures = self._exctract_raw_measures(filepath)

            self._phys_conditions[phys_cond_key][initial_time] = \
                measures

            current += 1
            status_display(current, files_total, phys_cond_key + " files scanned: ")


    def _exctract_raw_measures(self, filepath):
        measures = {}

        with open(filepath, 'r') as f:
            for line in f.readlines():
                matcher = FDSDataSource.PHYS_MEASURES_REGEXP.match(line)

                if matcher is not None:
                    point = (matcher.group(1), matcher.group(2))
                    value = matcher.group(3)
                    measures[point] = value

        # if not measures:
            # raise NoDataError('No physical measures in this period of time'
            #                  )

        return measures

    def _get_initial_time(self, filename):
        matcher = FDSDataSource.PHYS_FNAME_REGEXP.match(filename)
        return float(matcher.group(1))

    @property
    def filenames(self):
        return self._filenames


def show_loading_progress(current, total, msg):
    if total != 0:
    	percent = "%.1f" % (100 * current / total)
    	print(msg + percent + '%')
    else:
    	print('Noting to do.')


def extract_files_by_regexp(dir_path, regexp):
    """Finds file name matching defined regexp in a given directory.

    Omits directories, returns only paths pointing to regular files.
    """

    filenames = []
    for fname in listdir(dir_path):
        filepath = join(dir_path, fname)
        if isfile(filepath) and regexp.match(fname) is not None:
            filenames.append(fname)

    return filenames


def extract_line_by_regexp(file_path, regexp):
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


def root_out_none_values(list):
    return [el for el in list if el is not None]


class NoDataError(Exception):

    pass


class IncorrectDataError(Exception):

    pass


if __name__ == '__main__':
    src = GenericDataSource.factory('FDS','dane')
    src.parse_data()
    print(str(src.get_model_data()))
    print(str(src.phys_conditions))
