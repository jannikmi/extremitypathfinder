from collections.abc import Iterable, Mapping
from typing import TYPE_CHECKING, Any, Protocol

import networkx as nx
import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    # optional dependency (extra "shapely"), only imported when used
    from shapely.geometry.base import BaseGeometry

Coordinate = tuple[float, float]
Path = list[Coordinate]
Length = float | None
InputNumerical = float | int
InputCoord = tuple[InputNumerical, InputNumerical]
InputCoordList = npt.NDArray[Any] | list[InputCoord]
OptionalInputCoordList = InputCoordList | None
ObstacleIterator = Iterable[InputCoord]
if TYPE_CHECKING:
    Graph = nx.Graph[int]
else:
    # not subscriptable at runtime in all supported NetworkX versions
    Graph = nx.Graph
FloatArray = npt.NDArray[np.float64]


class GeoInterface(Protocol):
    """an object providing a GeoJSON-like mapping, e.g. a GeoPandas geometry"""

    @property
    def __geo_interface__(self) -> Mapping[str, Any]:
        """the GeoJSON-like mapping"""


# Shapely geometries, GeoJSON-like mappings (geometries, features, feature collections),
# objects with a ``__geo_interface__``, iterables of these or None (no geometry)
type GeometryInput = (
    BaseGeometry | Mapping[str, Any] | GeoInterface | Iterable[GeometryInput] | None
)
