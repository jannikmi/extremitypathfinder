import numpy as np

# stored in every pickled environment (``pickle``, ``copy``, multiprocessing). Increase it when the attributes change
# incompatibly, so that environments pickled with other versions are rejected with a clear error
PICKLE_FORMAT_VERSION = 6
# stored in every environment file (``PolygonEnvironment.save()``, cf. ``storage.py``). Unlike the pickle format it only
# changes with the graph semantics (which edges the visibility graph keeps), not with the internal attributes
FILE_FORMAT_VERSION = 1

# command line interface
# json data input format:
BOUNDARY_JSON_KEY = "boundary"
HOLES_JSON_KEY = "holes"


DTYPE_FLOAT = np.float64
