import heapq  # implementation of the heap queue algorithm, also known as the priority queue algorithm (binary tree)

# modified sample code from https://www.redblobgames.com/pathfinding/a-star/


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]  # return only the item without the priority


def modified_a_star(heuristic_graph, start, goal):
    """
    implementing the popular A* algorithm with optimisations for the special use case:
    IMPORTANT geometrical property of this problem (and hence also the extracted graph):

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

    :param heuristic_graph: the graph to search in
    :param start: the vertex to start from
    :param goal: the vertex to end at
    :return: a tuple of the shortest path start to goal and its total length
    """

    def put_in_queue(generator):
        try:
            # node, distance, cost estimate (= distance + heuristic = current-next + estimate(next-goal))
            next_n, dist, cost_estim = (next(generator))
            # the priority has to be the lower bound (=estimate) of the TOTAL cost!
            # = cost_so_far + cost_estim  (= start-current + estimate(current-goal))
            priority_queue.put((next_n, dist, generator, path, cost_so_far), cost_so_far + cost_estim)
        except StopIteration:
            # there is no neighbour left
            pass

    heuristic_graph.set_goal_node(goal)  # lazy update of the heuristic

    priority_queue = PriorityQueue()
    neighbour_gen = heuristic_graph._existing_edges_from(start)
    cost_so_far = 0.0
    path = [start]
    put_in_queue(neighbour_gen)
    visited_nodes = set()

    while not priority_queue.empty():
        # always 'visit' the node with the current lowest total cost estimate
        current, distance, neighbour_gen, path, cost_so_far = priority_queue.get()
        # print('visiting:', current)
        # print('neighbours:', heuristic_graph.get_neighbours_of(current))

        # there could still be other neighbours left in this generator:
        put_in_queue(neighbour_gen)

        if current in visited_nodes:
            # this node has already been visited. there is no need to consider
            # path can only get longer by visiting other nodes first: new_cost is never < cost_so_far
            continue

        visited_nodes.add(current)
        # NOTE: in contrast to vanilla A*, here the path and cost have to be stored separately for every open generator!
        cost_so_far += distance
        # IMPORTANT: use independent path lists
        path = path.copy()
        path.append(current)

        if current == goal:
            # since the goal node is the one with the lowest cost estimate
            # because of the geometric property mentioned above there can be no other shortest path to the goal
            # the algorithm can be terminated (regular a* would now still continue to all other neighbours first)
            # optimisation: _exiting_edges_from() returns the goal node first if it is among the neighbours
            # heuristic(goal) == 0
            # print('reached goal node. terminating')
            return path, cost_so_far

        # add neighbours of the current node
        neighbour_gen = heuristic_graph._existing_edges_from(current)
        put_in_queue(neighbour_gen)

    # goal is not reachable
    return [], None
