"""exact references for the floating point decisions of the package (rational arithmetic)

Every function decides exactly for the given float64 values (``fractions.Fraction`` represents them exactly).
Used by the tests and by ``prototypes/float_audit.py`` (#3).
"""

from fractions import Fraction


def _fractions(*points):
    return ([Fraction(float(c)) for c in p] for p in points)


def orientation(a, b, c) -> int:
    """the sign of the cross product (b - a) x (c - a): 1 counter clockwise, -1 clockwise, 0 collinear"""
    a, b, c = _fractions(a, b, c)
    cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return (cross > 0) - (cross < 0)


def rep(vector) -> Fraction | None:
    """the angle representation of ``_compute_repr_n_dist()`` (diamond angle per quadrant), ``None`` for (0, 0)"""
    ((dx, dy),) = _fractions(vector)
    if dx == 0 and dy == 0:
        return None
    if dx >= 0 and dy >= 0:
        quadrant, measure = 0, dy
    elif dx < 0 <= dy:
        quadrant, measure = 1, -dx
    elif dx < 0 and dy < 0:
        quadrant, measure = 2, -dy
    else:
        quadrant, measure = 3, dx
    return quadrant + measure / (abs(dx) + abs(dy))


def in_closure(direction, ray_start, ray_end) -> bool:
    """whether the direction lies counter clockwise from ``ray_start`` to ``ray_end`` (bounding rays included)"""
    start, end, d = rep(ray_start), rep(ray_end), rep(direction)
    if start == end:
        return True
    return (d - start) % 4 <= (end - start) % 4


def on_edge(p, p1, p2) -> bool:
    """whether the point lies on the edge p1-p2 (its vertices included)"""
    p, p1, p2 = _fractions(p, p1, p2)
    v1 = (p1[0] - p[0], p1[1] - p[1])
    v2 = (p2[0] - p[0], p2[1] - p[1])
    collinear = v1[0] * v2[1] - v1[1] * v2[0] == 0
    return collinear and v1[0] * v2[0] <= 0 and v1[1] * v2[1] <= 0


def inside_polygon(p, coords, border_value: bool) -> bool:
    """point in polygon test, ``border_value`` for points on the border"""
    nr_coords = len(coords)
    if any(on_edge(p, coords[i - 1], coords[i]) for i in range(nr_coords)):
        return border_value
    ((x, y),) = _fractions(p)
    inside = False
    for (x1, y1), (x2, y2) in zip(
        _fractions(*coords[-1:], *coords[:-1]), _fractions(*coords)
    ):
        if (y > y1) != (y > y2) and x1 + (y - y1) * (x2 - x1) / (y2 - y1) >= x:
            inside = not inside
    return inside


def signed_area2(coords) -> Fraction:
    """twice the signed area, negated: positive for clockwise numbering (cf. ``_has_clockwise_numbering()``)"""
    total = Fraction(0)
    for (x1, y1), (x2, y2) in zip(
        _fractions(*coords[-1:], *coords[:-1]), _fractions(*coords)
    ):
        total += (x2 - x1) * (y2 + y1)
    return total


def lies_behind(p1, p2, v) -> bool | None:
    """whether the vertex v lies behind the edge p1-p2 seen from the origin, ``None`` if parallel"""
    p1, p2, v = _fractions(p1, p2, v)
    a = p1[0] - p2[0]
    b = p1[1] - p2[1]
    det = a * v[1] - v[0] * b
    if det == 0:
        return None
    return (a * p1[1] - b * p1[0]) / det < 1
