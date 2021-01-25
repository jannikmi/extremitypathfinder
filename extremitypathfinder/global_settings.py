from typing import Iterable, List, Optional, Tuple, Union

import numpy as np

COORDINATE_TYPE = Tuple[float, float]
PATH_TYPE = List[COORDINATE_TYPE]
LENGTH_TYPE = Optional[float]
INPUT_NUMERICAL_TYPE = Union[float, int]
INPUT_COORD_TYPE = Tuple[INPUT_NUMERICAL_TYPE, INPUT_NUMERICAL_TYPE]
OBSTACLE_ITER_TYPE = Iterable[INPUT_COORD_TYPE]
INPUT_COORD_LIST_TYPE = Union[np.ndarray, List]
DEFAULT_PICKLE_NAME = "environment.pickle"

# command line interface
# json data input format:
BOUNDARY_JSON_KEY = "boundary"
HOLES_JSON_KEY = "holes"
