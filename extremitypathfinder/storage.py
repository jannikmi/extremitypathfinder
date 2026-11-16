"""The stable file format of prepared environments (``PolygonEnvironment.save()`` and ``load()``)

A ``.npz`` file (``numpy.savez_compressed``) that contains only data and is loaded without pickle:

========================= =========== ====================================================================
array                     type, shape content
========================= =========== ====================================================================
``format_version``        int, ()     ``configs.FILE_FORMAT_VERSION``
``boundary``              float, (n,2) the boundary polygon as stored by the environment (counter clockwise),
                                      shape (0,2) for an unbounded environment
``hole_sizes``            int, (h,)   the number of vertices of every hole
``hole_coords``           float, (v,2) the vertices of all holes (clockwise), one after the other
``extremity_indices``     int, (e,)   the graph nodes: indices into the vertices (boundary first, then the holes)
``edges``                 int, (m,2)  the edges of the visibility graph (two extremity indices each)
``weights``               float, (m,) the Euclidean lengths of the edges
========================= =========== ====================================================================

Everything else (the coordinates of all vertices, the mappings between vertices and edges, the points with their free
wedges) is computed from the polygons when loading (much faster than computing the visibility graph with Numba),
so the file survives changes of the internal data structures. Only a change of the graph semantics
(which edges the visibility graph keeps) needs a new ``configs.FILE_FORMAT_VERSION``.
The edges are stored in the order of the graph (``graph.edges``),
so that the loaded graph iterates its neighbours in the same order and the search breaks ties the same way.

Loading checks the structure of the file (cf. :func:`read` and :func:`build_graph`), but not that the graph is the
visibility graph of the polygons: that would need the preparation of the environment.
"""

import os
import zlib
from dataclasses import dataclass
from zipfile import BadZipFile

import numpy as np

from extremitypathfinder import configs, utils
from extremitypathfinder import types as t

_KEYS = (
    "format_version",
    "boundary",
    "hole_sizes",
    "hole_coords",
    "extremity_indices",
    "edges",
    "weights",
)


@dataclass(frozen=True)
class StoredEnvironment:
    """the content of a file, with a checked structure"""

    boundary: t.FloatArray | None
    holes: list[t.FloatArray]
    extremity_indices: t.IntArray
    edges: t.IntArray
    weights: t.FloatArray


def write(
    path: str | os.PathLike[str],
    boundary: t.FloatArray | None,
    holes: list[t.FloatArray],
    extremity_indices: t.IntArray,
    graph: t.Graph,
) -> None:
    """stores the polygons (as stored by the environment) and the visibility graph of their extremities"""
    # in the order of the graph, so that loading gives the same order of the neighbours
    edge_data: list[tuple[int, int, float]] = list(graph.edges(data="weight"))
    edges = np.array([(u, v) for u, v, _ in edge_data], dtype=np.int64).reshape(-1, 2)
    weights = np.array([weight for _, _, weight in edge_data], dtype=np.float64)
    boundary_array = (
        np.empty((0, 2), dtype=np.float64) if boundary is None else boundary
    )
    hole_coords = np.concatenate(holes) if holes else np.empty((0, 2), dtype=np.float64)
    # a file object: ``savez_compressed`` appends ".npz" to a path without this extension
    with open(path, "wb") as f:
        np.savez_compressed(
            f,
            format_version=np.array(configs.FILE_FORMAT_VERSION, dtype=np.int64),
            boundary=boundary_array,
            hole_sizes=np.array([len(hole) for hole in holes], dtype=np.int64),
            hole_coords=hole_coords,
            extremity_indices=np.asarray(extremity_indices, dtype=np.int64),
            edges=edges,
            weights=weights,
        )


def _check_array(
    arrays: dict[str, np.ndarray], name: str, kind: str, ndim: int, columns: int = 0
) -> None:
    array = arrays[name]
    # ``kind``: "i" integer, "f" floating point
    if array.dtype.kind != kind:
        expected = "integer" if kind == "i" else "floating point"
        raise ValueError(f"'{name}' must be {expected}, got {array.dtype}.")
    if array.ndim != ndim or (columns and array.shape[1] != columns):
        shape = f"{ndim} dimensions" + (f" with {columns} columns" if columns else "")
        raise ValueError(f"'{name}' must have {shape}, got shape {array.shape}.")


def _load_arrays(path: str | os.PathLike[str]) -> dict[str, np.ndarray]:
    """the arrays of the file that belong to the format, without unpickling anything

    :raises OSError: when the file cannot be opened (e.g. it does not exist)
    :raises ValueError: when the content cannot be read as a ``.npz`` file
    """
    # opened here: errors of the file system stay OSErrors, everything raised while reading the content is a ValueError
    with open(path, "rb") as f:
        try:
            with np.load(f, allow_pickle=False) as file:
                names = set(file.files)
                return {key: file[key] for key in _KEYS if key in names}
        except ValueError as error:
            # numpy refuses object arrays (pickled data) and files that are no ``.npz`` files, e.g. a pickle file
            raise ValueError(
                f"{os.fspath(path)}: not an environment file ({error})"
            ) from None
        except (
            BadZipFile,
            EOFError,
            OSError,
            RuntimeError,
            zlib.error,
        ) as error:
            # a corrupted or truncated zip file: a CRC mismatch, an invalid compressed stream or zip header
            raise ValueError(
                f"{os.fspath(path)}: corrupted or truncated file ({type(error).__name__}: {error})."
            ) from None


def read(path: str | os.PathLike[str]) -> StoredEnvironment:
    """reads a file written by :func:`write` and checks its structure

    The polygons are not validated (they were validated when the environment was created).

    :raises ValueError: for a file that is no ``.npz`` file (also for a pickle file) or does not contain all arrays,
        is corrupted, has an unknown format version, arrays of the wrong type or shape,
        or no polygon at all, or polygons that are not finite, have fewer than 3 vertices or are not stored in the
        orientation of the environment (counter clockwise boundary, clockwise holes)
    """
    arrays = _load_arrays(path)
    missing = [key for key in _KEYS if key not in arrays]
    if missing:
        raise ValueError(
            f"{os.fspath(path)}: not an environment file, missing the arrays {missing}."
        )

    version = arrays["format_version"]
    if version.shape != () or version.dtype.kind != "i":
        raise ValueError("'format_version' must be a single integer.")
    if int(version) != configs.FILE_FORMAT_VERSION:
        raise ValueError(
            f"The environment file has format version {int(version)}, "
            f"this version of extremitypathfinder reads format version {configs.FILE_FORMAT_VERSION}. "
            "Recreate the environment from its polygons and save it again."
        )
    _check_array(arrays, "boundary", "f", 2, 2)
    _check_array(arrays, "hole_sizes", "i", 1)
    _check_array(arrays, "hole_coords", "f", 2, 2)
    _check_array(arrays, "extremity_indices", "i", 1)
    _check_array(arrays, "edges", "i", 2, 2)
    _check_array(arrays, "weights", "f", 1)
    hole_sizes = arrays["hole_sizes"]
    hole_coords = arrays["hole_coords"]
    if np.any(hole_sizes < 3) or int(hole_sizes.sum()) != len(hole_coords):
        raise ValueError(
            "'hole_sizes' must be at least 3 each and sum up to the number of 'hole_coords'."
        )
    if len(arrays["weights"]) != len(arrays["edges"]):
        raise ValueError("'weights' and 'edges' must have the same length.")

    # shape (0,2): no boundary polygon (a polygon has at least 3 vertices)
    boundary = arrays["boundary"] if len(arrays["boundary"]) else None
    if boundary is None and not len(hole_sizes):
        raise ValueError(
            "The file contains no polygon: an unbounded environment needs at least one hole."
        )
    # (``np.split`` of no holes would give one empty hole)
    holes = (
        [hole.copy() for hole in np.split(hole_coords, np.cumsum(hole_sizes)[:-1])]
        if len(hole_sizes)
        else []
    )
    try:
        utils.check_polygon_formats(boundary, holes)
    except (TypeError, ValueError) as error:
        raise ValueError(str(error)) from None
    # the vertex indices of the edges refer to this numbering
    if boundary is not None and utils.orient_polygon(boundary, False) is not boundary:
        raise ValueError(
            "The boundary polygon must be numbered counter clockwise (as stored by the environment)."
        )
    for hole_idx, hole in enumerate(holes):
        if utils.orient_polygon(hole, True) is not hole:
            raise ValueError(
                f"hole {hole_idx} must be numbered clockwise (as stored by the environment)."
            )
    return StoredEnvironment(
        boundary,
        holes,
        arrays["extremity_indices"],
        arrays["edges"],
        arrays["weights"],
    )


def build_graph(stored: StoredEnvironment, extremity_indices: t.IntArray) -> t.Graph:
    """the visibility graph of a file, checked against the extremities of its polygons

    :param extremity_indices: the extremities computed from the polygons of the file
    :raises ValueError: when the extremities of the file differ from the computed ones,
        an edge has an end that is no extremity, connects an extremity with itself or occurs twice,
        or a weight is not finite and positive
    """
    if not np.array_equal(stored.extremity_indices, extremity_indices):
        raise ValueError(
            "The extremities of the file differ from those of its polygons: "
            "the file is corrupted or was written for other polygons."
        )
    edges = stored.edges
    if len(edges):
        if not np.all(np.isin(edges, extremity_indices)):
            raise ValueError(
                "'edges' must only connect extremities (indices out of range or no extremity)."
            )
        if np.any(edges[:, 0] == edges[:, 1]):
            raise ValueError("'edges' must not connect an extremity with itself.")
        unique = np.unique(np.sort(edges, axis=1), axis=0)
        if len(unique) != len(edges):
            raise ValueError("'edges' must not contain an edge twice.")
        if not np.all(np.isfinite(stored.weights) & (stored.weights > 0.0)):
            raise ValueError("'weights' must be finite and positive.")
    graph = t.Graph()
    # all extremities, also those without edges
    graph.add_nodes_from(extremity_indices.tolist())
    graph.add_weighted_edges_from(
        zip(edges[:, 0].tolist(), edges[:, 1].tolist(), stored.weights.tolist())
    )
    return graph
