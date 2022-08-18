import heapq
from typing import Dict, Iterable, Iterator, List, Optional, Set, Tuple

import numpy as np


class SearchState(object):
    __slots__ = ["node", "distance", "neighbours", "path", "cost_so_far", "priority"]

    def __init__(self, node, distance, neighbour_generator, path, cost_so_far, cost_estim):
        self.node = node
        self.distance = distance
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

    def join_identical(self):
        # for shortest path computations all graph nodes should be unique
        # join all nodes with the same coordinates
        # leave dangling nodes! (they might become reachable by adding start and and goal node!)
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
