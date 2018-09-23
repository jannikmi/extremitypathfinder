import numpy as np

'''
requirements for the data:
 - no self intersections
 - no neighbouring identical edges
 - expected edge numbering:
    outer boundary polygon: counter clockwise
    holes: clockwise
    
'''


class AngleRepresentation:
    quadrant = None
    angle_measure = None
    representation = None

    def __init__(self, np_vector):
        # 2D vector: (dx, dy) = np_vector
        norm = np.linalg.norm(np_vector)
        if norm == 0.0:
            # TODO make sure norm is not 0!
            raise ValueError(norm)

        dx_positive = np_vector[0] >= 0
        dy_positive = np_vector[1] >= 0

        if dx_positive and dy_positive:
            self.quadrant = 0.0
            self.angle_measure = np_vector[1] / norm

        elif not dx_positive and dy_positive:
            self.quadrant = 1.0
            self.angle_measure = -np_vector[0] / norm

        elif not dx_positive and not dy_positive:
            self.quadrant = 2.0
            self.angle_measure = -np_vector[1] / norm

        else:
            self.quadrant = 3.0
            self.angle_measure = np_vector[0] / norm

        self.representation = self.quadrant + self.angle_measure


def find_extremities(polygon):
    """
    :param polygon: a list of numpy 2D coordinates (arrays, vectors) representing the edges of a polygon.
    expected edge numbering:
    outer boundary polygon: counter clockwise
    holes: clockwise
    :return: a list of indices from all points in the polygon with an outer angle of > 180 degree
    """

    polygon_length = len(polygon)
    if polygon_length < 3:
        raise ValueError('This is not a valid polygon: # edges:', polygon_length)
    extremity_indices = []
    extremity_index = -1
    p1 = polygon[-2]
    p2 = polygon[-1]

    for p3 in polygon:

        if AngleRepresentation(p3 - p2).representation - AngleRepresentation(p1 - p2).representation % 4 < 2.0:
            # %4 because the quadrant has to be in [0,1,2,3] (representation in [0:4[)
            # outer angle between p1p2p3 is > 180 degree
            # at first the polygon index is -1 so
            extremity_indices.append(extremity_index % polygon_length)

        # move to the next point
        p1 = p2
        p2 = p3
        extremity_index += 1

    return extremity_indices


# preparation of the map
# TODO fct. for ensuring clockwise and counter clockwise numbering


class Vertex:
    polygon_id = None
    id = None
    coordinates = None
    is_extremity = False

    def __init__(self, polygon_id, id, coordinates):
        self.polygon_id = polygon_id
        self.id = id
        self.coordinates = coordinates

    def declare_extremity(self):
        self.is_extremity = True


class Edge:
    polygon_id = None
    id = None

    def __init__(self, polygon_id, id):
        self.polygon_id = polygon_id
        self.id = id





# compute the angle representations and distances for all vertices respective to the query point

# when the query point is an extremity itself:
# neighbouring extremities are reachable with the distance equal to the edge length
# do not check the neighbouring edges
# eliminate all extremities 'behind' the query point from the candidate set




# for all candidate edges check if there are extremities (besides the ones belonging to the edge) lying within this angle

# eliminate all extremities lying 'behind' the edge from the candidate set
# extremities directly on the edge are allowed

# check the neighbouring edges of all extremities which lie in front of the edge next first (prioritize them)
# they lie in front and hence will eliminate other extremities faster




# TODO class for keeping preloaded map
class MapStorage:
    boundary_polygon = None
    holes = None

    boundary_extremities = None
    hole_extremities = None

    def load_map(self, boundary_polygon, holes, validation=False):
        # 'loading the map
        self.boundary_polygon = boundary_polygon
        self.holes = holes
        if validation:
            # TODO validating polygons
            # TODO rectification
            #  - no self intersections
            #  - no neighbouring identical edges
            #  - expected edge numbering:
            #     outer boundary polygon: counter clockwise
            #     holes: clockwise
            pass

    def prepare(self):
        # find and store the extremities of the boundary polygon and all the holes
        self.boundary_extremities = find_extremities(self.boundary_polygon)

        self.hole_extremities = []
        for hole in self.holes:
            self.hole_extremities.append(find_extremities(hole))

        # compute the all directly reachable extremities based on visibility
        # compute the distances between all directly reachable extremities
        # TODO ? compute shortest paths between all directly reachable extremities
        # store as graph
        # TODO matrix? sparse matrix data type? look up what a star implementation needs

# path planning query:
# make sure start and goal are within the boundary polygon and outside of all holes

# compute the all directly reachable extremities from start and goal based on visibility

# a star algorithm
# TODO more clever approach
