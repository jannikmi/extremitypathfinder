import heapq
from typing import Dict, Iterable, Iterator, List, Optional, Set, Tuple

import numpy as np

# TODO find a way to avoid global variable, wrap all in a different kind of 'coordinate system environment'?
# problem: lazy evaluation, passing the origin every time is not an option
# placeholder for temporarily storing the origin of the current coordinate system

origin = None


def compute_angle_repr_inner(np_vector: np.ndarray) -> float:
    """computing representation for the angle from the origin to a given vector

    value in [0.0 : 4.0[
    every quadrant contains angle measures from 0.0 to 1.0
    there are 4 quadrants (counter clockwise numbering)
    0 / 360 degree -> 0.0
    90 degree -> 1.0
    180 degree -> 2.0
    270 degree -> 3.0
    ...
    Useful for comparing angles without actually computing expensive trigonometrical functions
    This representation does not grow directly proportional to its represented angle,
    but it its bijective and monotonous:
    rep(p1) > rep(p2) <=> angle(p1) > angle(p2)
    rep(p1) = rep(p2) <=> angle(p1) = angle(p2)
    angle(p): counter clockwise angle between the two line segments (0,0)'--(1,0)' and (0,0)'--p
    with (0,0)' being the vector representing the origin

    :param np_vector:
    :return:
    """
    # 2D vector: (dx, dy) = np_vector
    dx, dy = np_vector
    dx_positive = dx >= 0
    dy_positive = dy >= 0

    if dx_positive and dy_positive:
        quadrant = 0.0
        angle_measure = dy

    elif not dx_positive and dy_positive:
        quadrant = 1.0
        angle_measure = -dx

    elif not dx_positive and not dy_positive:
        quadrant = 2.0
        angle_measure = -dy

    else:
        quadrant = 3.0
        angle_measure = dx

    norm = np.linalg.norm(np_vector, ord=2)
    if norm == 0.0:
        # make sure norm is not 0!
        raise ValueError("received null vector:", np_vector, norm)
    # normalise angle measure to [0; 1]
    angle_measure /= norm
    return quadrant + angle_measure


class AngleRepresentation(object):
    # prevent dynamic attribute assignment (-> safe memory)
    # __slots__ = ['quadrant', 'angle_measure', 'value']
    __slots__ = ["value"]

    def __init__(self, np_vector):
        self.value = compute_angle_repr_inner(np_vector)

    def __str__(self):
        return str(self.value)

    def __repr__(self):
        return self.__str__()


class Vertex(object):
    # defining static attributes on class to safe memory
    __slots__ = [
        "coordinates",
        "is_extremity",
        "is_outdated",
        "coordinates_translated",
        "angle_representation",
        "distance_to_origin",
    ]

    def __init__(self, coordinates):
        self.coordinates = np.array(coordinates)
        self.is_extremity: bool = False

        # a container for temporally storing shifted coordinates
        self.coordinates_translated = None
        self.angle_representation: Optional[AngleRepresentation] = None
        self.distance_to_origin: float = 0.0

        # for lazy evaluation: often the angle representations dont have to be computed for every vertex!
        self.is_outdated: bool = True

    def __gt__(self, other):
        # ordering needed for priority queue. multiple vertices possibly have the same priority.
        return id(self) > id(other)

    def __str__(self):
        return str(tuple(self.coordinates))

    def __repr__(self):
        return self.__str__()

    def evaluate(self):
        global origin
        # store the coordinate value of the point relative to the new origin vector
        self.coordinates_translated = self.coordinates - origin.coordinates
        self.distance_to_origin = np.linalg.norm(self.coordinates_translated)
        if self.distance_to_origin == 0.0:
            # the coordinates of the origin and this vertex are equal
            # an angle is not defined in this case!
            self.angle_representation = None
        else:
            self.angle_representation = AngleRepresentation(self.coordinates_translated)

        self.is_outdated = False

    def get_coordinates_translated(self):
        if self.is_outdated:
            self.evaluate()
        return self.coordinates_translated

    def get_angle_representation(self):
        if self.is_outdated:
            self.evaluate()
        try:
            return self.angle_representation.value
        except AttributeError:
            return None

    def get_distance_to_origin(self):
        if self.is_outdated:
            self.evaluate()
        return self.distance_to_origin

    def mark_outdated(self):
        self.is_outdated = True


def compute_angle_repr(coords_v1: np.ndarray, coords_v2: np.ndarray) -> Optional[float]:
    diff_vect = coords_v2 - coords_v1
    if np.all(diff_vect == 0.0):
        return None
    return compute_angle_repr_inner(diff_vect)


def angle_rep_inverse(repr: Optional[float]) -> Optional[float]:
    if repr is None:
        repr_inv = None
    else:
        repr_inv = (repr + 2.0) % 4.0
    return repr_inv


class PolygonVertex(Vertex):
    # __slots__ declared in parents are available in child classes. However, child subclasses will get a __dict__
    # and __weakref__ unless they also define __slots__ (which should only contain names of any additional slots).
    __slots__ = ["edge1", "edge2", "neighbour1", "neighbour2"]

    def __init__(self, *args, **kwargs):
        super(PolygonVertex, self).__init__(*args, **kwargs)
        self.edge1: Optional[Edge] = None
        self.edge2: Optional[Edge] = None
        self.neighbour1: Optional[PolygonVertex] = None
        self.neighbour2: Optional[PolygonVertex] = None

    def get_neighbours(self):
        return self.neighbour1, self.neighbour2

    def declare_extremity(self):
        self.is_extremity = True

    def set_edge1(self, e1):
        self.edge1 = e1
        # ordering is important! the numbering convention has to stay intact!
        self.neighbour1 = e1.vertex1

    def set_edge2(self, e2):
        self.edge2 = e2
        # ordering is important! the numbering convention has to stay intact!
        self.neighbour2 = e2.vertex2


class Edge(object):
    __slots__ = ["vertex1", "vertex2"]

    def __init__(self, vertex1, vertex2):
        self.vertex1: PolygonVertex = vertex1
        self.vertex2: PolygonVertex = vertex2

    def __str__(self):
        return self.vertex1.__str__() + "-->" + self.vertex2.__str__()

    def __repr__(self):
        return self.__str__()


class Polygon(object):
    __slots__ = ["vertices", "edges", "coordinates", "is_hole", "_extremities"]

    def __init__(self, coordinate_list, is_hole):
        # store just the coordinates separately from the vertices in the format suiting the inside_polygon() function
        self.coordinates = np.array(coordinate_list)

        self.is_hole: bool = is_hole

        if len(coordinate_list) < 3:
            raise ValueError(
                "This is not a valid polygon:",
                coordinate_list,
                "# edges:",
                len(coordinate_list),
            )

        self.vertices: List[PolygonVertex] = [PolygonVertex(coordinate) for coordinate in coordinate_list]

        self.edges: List[Edge] = []
        vertex1 = self.vertices[-1]
        for vertex2 in self.vertices:
            edge = Edge(vertex1, vertex2)
            # ordering is important! the numbering convention has to stay intact!
            vertex1.set_edge2(edge)
            vertex2.set_edge1(edge)
            self.edges.append(edge)
            vertex1 = vertex2

        self._extremities: Optional[List[PolygonVertex]] = None

    def _find_extremities(self):
        """
        identify all protruding points = vertices with an inside angle of > 180 degree ('extremities')
        expected edge numbering:
            outer boundary polygon: counter clockwise
            holes: clockwise
        :return:
        """
        extremities = []
        # extremity_indices = []
        # extremity_index = -1
        v1 = self.vertices[-2]
        p1 = v1.coordinates
        v2 = self.vertices[-1]
        p2 = v2.coordinates

        for v3 in self.vertices:
            p3 = v3.coordinates
            # since consequent vertices are not permitted to be equal,
            #   the angle representation of the difference is well defined
            diff_p3_p2 = p3 - p2
            # TODO optimise
            diff_p1_p2 = p1 - p2

            if (AngleRepresentation(diff_p3_p2).value - AngleRepresentation(diff_p1_p2).value) % 4 < 2.0:
                # basic idea:
                #   - translate the coordinate system to have p2 as origin
                #   - compute the angle representations of both vectors representing the edges
                #   - "rotate" the coordinate system (equal to deducting) so that the p1p2 representation is 0
                #   - check in which quadrant the p2p3 representation lies
                # %4 because the quadrant has to be in [0,1,2,3] (representation in [0:4[)
                # if the representation lies within quadrant 0 or 1 (<2.0), the inside angle
                #   (for boundary polygon inside, for holes outside) between p1p2p3 is > 180 degree
                # then p2 = extremity
                v2.declare_extremity()
                extremities.append(v2)

            # move to the next point
            # vertex1=vertex2
            # TODO optimise
            diff_p1_p2 = diff_p3_p2
            p1 = p2
            v2 = v3
            p2 = p3

        self._extremities = extremities

    @property
    def extremities(self) -> List[PolygonVertex]:
        if self._extremities is None:
            self._find_extremities()
        return self._extremities

    def translate(self, new_origin: Vertex):
        global origin
        origin = new_origin
        for vertex in self.vertices:
            vertex.mark_outdated()


class SearchState(object):
    __slots__ = ["node", "distance", "neighbours", "path", "cost_so_far", "priority"]

    def __init__(self, node, distance, neighbour_generator, path, cost_so_far, cost_estim):
        self.node = node
        self.distance = distance
        # TODO
        self.neighbours = neighbour_generator
        self.path = path
        self.cost_so_far: float = cost_so_far
        # the priority has to be the lower bound (=estimate/"heuristic") of the TOTAL cost!
        # = cost_so_far + cost_estim  (= start-current + estimate(current-goal))
        self.priority: float = cost_so_far + cost_estim

    def __lt__(self, other):  # defines an ordering -> items can be stored in a sorted heap
        return self.priority < other.priority


class SearchStateQueue(object):
    def __init__(self):
        self.heap_elements: List[SearchState] = []

    def is_empty(self) -> bool:
        return len(self.heap_elements) == 0

    def put(self, item: SearchState) -> None:
        heapq.heappush(self.heap_elements, item)

    def get(self):
        s = heapq.heappop(self.heap_elements)
        return s.node, s.neighbours, s.distance, s.path, s.cost_so_far


def get_distance_to_origin(coords_origin: np.ndarray, coords_v: np.ndarray) -> float:
    coords_rel = coords_v - coords_origin
    return np.linalg.norm(coords_rel, ord=2)


NodeId = int


# TODO often empty sets in self.neighbours
class DirectedHeuristicGraph(object):
    __slots__ = ["all_nodes", "distances", "goal_coords", "heuristic", "neighbours", "coord_map", "merged_id_mapping"]

    def __init__(self, coord_map: Optional[Dict[NodeId, np.ndarray]] = None):
        self.distances: Dict[Tuple[NodeId, NodeId], float] = {}
        self.neighbours: Dict[NodeId, Set[NodeId]] = {}

        if coord_map is None:
            all_nodes = set()
        else:
            all_nodes = set(coord_map.keys())

        self.all_nodes: Set[NodeId] = set(all_nodes)  # independent copy required!
        self.coord_map: Dict[NodeId, np.ndarray] = coord_map
        self.merged_id_mapping: Dict[NodeId, NodeId] = {}

        # TODO use same set as extremities of env, but different for copy!

        # the heuristic must NEVER OVERESTIMATE the actual cost (here: actual shortest distance)
        # <=> must always be lowest for node with the POSSIBLY lowest cost
        # <=> heuristic is LOWER BOUND for the cost
        # the heuristic here: distance to the goal node (is always the shortest possible distance!)
        self.heuristic: Dict[NodeId, float] = {}
        self.goal_coords: Optional[np.ndarray] = None

    def __deepcopy__(self, memodict=None):
        # returns an independent copy (nodes can be added without changing the original graph),
        # but without actually copying vertex instances!
        independent_copy = DirectedHeuristicGraph()
        independent_copy.distances = self.distances.copy()
        independent_copy.neighbours = {k: v.copy() for k, v in self.neighbours.items()}
        independent_copy.all_nodes = self.all_nodes.copy()
        return independent_copy

    def get_all_nodes(self) -> Set[NodeId]:
        return self.all_nodes

    def get_neighbours(self) -> Iterable:
        yield from self.neighbours.items()

    def get_neighbours_of(self, node: NodeId) -> Set[NodeId]:
        return self.neighbours.get(node, set())

    def get_distance(self, node1: NodeId, node2: NodeId) -> float:
        # directed edges: just one direction is being stored
        return self.distances[(node1, node2)]

    def get_heuristic(self, node: NodeId) -> float:
        # lazy evaluation:
        h = self.heuristic.get(node, None)
        if h is None:
            # has been reset, compute again
            h = get_distance_to_origin(self.goal_coords, self.coord_map[node])
            self.heuristic[node] = h
        return h

    def gen_neighbours_and_dist(self, node1: NodeId) -> Iterator[Tuple[NodeId, float, float]]:
        # optimisation:
        #   return the neighbours ordered after their cost estimate: distance+ heuristic (= current-next + next-goal)
        #   -> when the goal is reachable return it first (-> a star search terminates)
        neighbours = self.get_neighbours_of(node1)
        distances = [self.get_distance(node1, n) for n in neighbours]

        def entry_generator(neighbours, distances):
            for node2, distance in zip(neighbours, distances):
                yield node2, distance, distance + self.get_heuristic(node2)

        out_sorted = sorted(entry_generator(neighbours, distances), key=lambda x: x[2])

        # yield node, distance, cost_estimate= distance + heuristic
        yield from out_sorted

    def add_directed_edge(self, node1: NodeId, node2: NodeId, distance: float):
        assert node1 != node2  # no self loops allowed!
        self.neighbours.setdefault(node1, set()).add(node2)
        self.distances[(node1, node2)] = distance
        self.all_nodes.add(node1)
        self.all_nodes.add(node2)

    def add_undirected_edge(self, node1: NodeId, node2: NodeId, distance: float):
        assert node1 != node2  # no self loops allowed!
        self.add_directed_edge(node1, node2, distance)
        self.add_directed_edge(node2, node1, distance)

    def add_multiple_undirected_edges(self, node1: NodeId, node_distance_map: Dict[NodeId, float]):
        for node2, distance in node_distance_map.items():
            self.add_undirected_edge(node1, node2, distance)

    def add_multiple_directed_edges(self, node1: NodeId, node_distance_iter: Dict[NodeId, float]):
        for node2, distance in node_distance_iter.items():
            self.add_directed_edge(node1, node2, distance)

    def remove_directed_edge(self, n1: NodeId, n2: NodeId):
        neighbours = self.get_neighbours_of(n1)
        neighbours.discard(n2)
        self.distances.pop((n1, n2), None)
        # ATTENTION: even if there are no neighbours left and a node is hence dangling (not reachable),
        # the node must still be kept, since with the addition of start and goal nodes during a query
        # the node might become reachable!

    def remove_undirected_edge(self, node1: NodeId, node2: NodeId):
        # should work even if edge does not exist yet
        self.remove_directed_edge(node1, node2)
        self.remove_directed_edge(node2, node1)

    def remove_multiple_undirected_edges(self, node1: NodeId, node2_iter: Iterable[NodeId]):
        for node2 in node2_iter:
            self.remove_undirected_edge(node1, node2)

    def make_clean(self):
        # for shortest path computations all graph nodes should be unique
        self.join_identical()
        # leave dangling nodes! (they might become reachable by adding start and and goal node!)

    def join_identical(self):
        # join all nodes with the same coordinates,
        nodes_to_check = self.all_nodes.copy()
        while len(nodes_to_check) > 1:
            n1 = nodes_to_check.pop()
            coordinates1 = self.coord_map[n1]
            same_nodes = {n for n in nodes_to_check if np.allclose(coordinates1, self.coord_map[n])}
            nodes_to_check.difference_update(same_nodes)
            for n2 in same_nodes:
                self.merge_nodes(n1, n2)

    def remove_node(self, n: NodeId):
        self.all_nodes.discard(n)
        # also deletes all edges
        # outgoing
        neighbours = self.neighbours.pop(n, set())
        for n1 in neighbours:
            self.distances.pop((n, n1), None)
            self.distances.pop((n1, n), None)
        # incoming
        for n1 in self.all_nodes:
            neighbours = self.neighbours.get(n1, set())
            neighbours.discard(n)
            self.distances.pop((n, n1), None)
            self.distances.pop((n1, n), None)

    def merge_nodes(self, n1: NodeId, n2: NodeId):
        # print('removing duplicate node', n2)
        neighbours_n1 = self.neighbours.get(n1, set())
        neighbours_n2 = self.neighbours.pop(n2, {})
        for n3 in neighbours_n2:
            d = self.distances.pop((n2, n3))
            self.distances.pop((n3, n2), None)
            self.neighbours.get(n3, set()).discard(n2)
            # do not allow self loops!
            if n3 != n1 and n3 not in neighbours_n1:
                # and add all the new edges to node 1
                self.add_undirected_edge(n1, n3, d)

        self.remove_node(n2)
        self.merged_id_mapping[n2] = n1  # mapping from -> to

    def modified_a_star(self, start: int, goal: int, goal_coords: np.ndarray) -> Tuple[List[int], Optional[float]]:
        """implementation of the popular A* algorithm with optimisations for this special use case

        IMPORTANT: geometrical property of this problem (and hence also the extracted graph):
        it is always shortest to directly reach a node instead of visiting other nodes first
        (there is never an advantage through reduced edge weight)

        this can be exploited in a lot of cases to make A* faster and terminate earlier than for general graphs:
        -> when the goal is directly reachable, there can be no other shorter path to it. terminate.
        -> there is no need to revisit nodes (path only gets longer)
        -> not all neighbours of the current node have to be checked like in vanilla A* before continuing
        to the next node.

        Optimisation: keep one 'neighbour generator' open for every visited node,
            that is yielding its neighbours starting with the one having the lowest cost estimate.
            One then only has to store those generators in a priority queue
            to always draw the generator with the neighbour having the lowest TOTAL cost estimate (lower bound) next.

        modified sample code from https://www.redblobgames.com/pathfinding/a-star/

        Terminology:
        search progress: start -> last -> current -> goal (nodes)
        heuristic: lower bound of the distance from current to goal
        distance: from last to current
        cost_estimate: distance + heuristic, used to prioritise the neighbours to check
        cost_so_far: length of the path from start to last node
        priority: cost_so_far + cost_estimate, used to prioritise all possible search states (<- sorting of heap queue!)

        :param start: the vertex to start from
        :param goal: the vertex to end at
        :return: a tuple of the shortest path from start to goal and its total length.
            ([], None) if there is no possible path.
        """

        # apply mapping in case start or goal got merged with another node
        start = self.merged_id_mapping.get(start, start)
        goal = self.merged_id_mapping.get(goal, goal)

        def enqueue(neighbours: Iterator):
            try:
                next_node, distance, cost_estim = next(neighbours)
            except StopIteration:
                # there is no neighbour left
                return
            state = SearchState(next_node, distance, neighbours, path, cost_so_far, cost_estim)
            search_state_queue.put(state)

        self.goal_coords = goal_coords  # lazy update of the heuristic

        search_state_queue = SearchStateQueue()
        current_node = start
        neighbours = self.gen_neighbours_and_dist(current_node)
        cost_so_far = 0.0
        path = [start]
        enqueue(neighbours)
        visited_nodes = set()

        while not search_state_queue.is_empty():
            # always 'visit' the node with the current lowest estimated TOTAL cost (not! heuristic)
            (
                current_node,
                neighbours,
                distance,
                path,
                cost_so_far,
            ) = search_state_queue.get()
            # print('visiting:', current_node)
            # print('neighbours:', heuristic_graph.get_neighbours_of(current_node))

            # there could still be other neighbours left in this generator:
            enqueue(neighbours)

            if current_node in visited_nodes:
                # this node has already been visited. there is no need to consider
                # path can only get longer by visiting other nodes first: new_cost is never < cost_so_far
                continue

            visited_nodes.add(current_node)
            # NOTE: in contrast to vanilla A*,
            # here the path and cost have to be stored separately for every open generator!
            cost_so_far += distance
            # IMPORTANT: use independent path lists
            path = path.copy()
            path.append(current_node)

            if current_node == goal:
                # since the goal node is the one with the lowest cost estimate
                # because of the geometric property mentioned above there can be no other shortest path to the goal
                # the algorithm can be terminated (regular a* would now still continue to all other neighbours first)
                # optimisation: _exiting_edges_from() returns the goal node first if it is among the neighbours
                # heuristic(goal) == 0
                # print('reached goal node. terminating')
                return path, cost_so_far

            # also consider the neighbours of the current node
            neighbours = self.gen_neighbours_and_dist(current_node)
            enqueue(neighbours)

        # goal is not reachable
        return [], None
