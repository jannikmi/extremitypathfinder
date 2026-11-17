from collections.abc import Iterable, Mapping
from typing import TYPE_CHECKING, Any, Protocol

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
# the visibility graph: node -> neighbour -> edge weight (Euclidean distance), every edge in both directions
type Graph = dict[int, dict[int, float]]
FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int64]
# the state of a pickled environment: the attributes, and the ``__slots__`` of subclasses as a second dict
type PickleState = dict[str, object] | tuple[dict[str, object], dict[str, object]]


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
