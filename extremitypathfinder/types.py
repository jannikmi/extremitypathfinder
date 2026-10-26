from collections.abc import Iterable

import networkx as nx
import numpy as np

Coordinate = tuple[float, float]
Path = list[Coordinate]
Length = float | None
InputNumerical = float | int
InputCoord = tuple[InputNumerical, InputNumerical]
InputCoordList = np.ndarray | list[InputCoord]
OptionalInputCoordList = InputCoordList | None
ObstacleIterator = Iterable[InputCoord]
Graph = nx.Graph
