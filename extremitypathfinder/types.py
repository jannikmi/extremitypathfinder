from typing import Iterable, List, Optional, Tuple, Union

import networkx as nx
import numpy as np

Coordinate = Tuple[float, float]
Path = List[Coordinate]
Length = Optional[float]
InputNumerical = Union[float, int]
InputCoord = Tuple[InputNumerical, InputNumerical]
InputCoordList = Union[np.ndarray, List[InputCoord]]
ObstacleIterator = Iterable[InputCoord]
Graph = nx.Graph
