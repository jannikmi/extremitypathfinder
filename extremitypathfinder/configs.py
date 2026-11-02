import numpy as np

DEFAULT_PICKLE_NAME = "environment.pickle"
# stored in every pickled environment. Increase it when the attributes change incompatibly,
# so that environments pickled with other versions are rejected with a clear error
PICKLE_FORMAT_VERSION = 4

# command line interface
# json data input format:
BOUNDARY_JSON_KEY = "boundary"
HOLES_JSON_KEY = "holes"


DTYPE_FLOAT = np.float64
