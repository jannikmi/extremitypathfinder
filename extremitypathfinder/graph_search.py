import heapq  # implementation of the heap queue algorithm, also known as the priority queue algorithm (binary tree)
from math import inf as infinity

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
        this can be exploited in a lot of cases to make a* terminate earlier than for general graphs
        -> when the goal is directly reachable, there can be no other shorter path to it. Terminate
        -> when always only expanding the nodes with the lowest estimated cost (lower bound),
         there is no need to revisit nodes (path only gets longer)
    :param heuristic_graph: the graph to search in
    :param start: the vertex to start from
    :param goal: the vertex to end at
    :return: a tuple of the shortest path FROM GOAL TO START (reversed) and its total length
    """

    def reconstruct_path():
        # backtrack where path came from
        current_node = goal
        path = []
        while current_node is not None:
            path.append(current_node)
            current_node = came_from[current_node]
        # path.reverse()  # to get the actual path
        return path

    heuristic_graph.set_goal_node(goal)  # lazy update of the heuristic

    # idea: make sure to not 'walk back' delete edges with increasing heuristic?!
    #   -> not worth it. requires basically a a* search...

    priority_queue = PriorityQueue()
    priority_queue.put(start, 0.0)
    came_from = {start: None}
    cost_so_far = {start: 0.0}
    expanded_nodes = set()

    while not priority_queue.empty():
        # always 'expand' the node with the lowest current cost estimate (= cost_so_far + heuristic)
        current = priority_queue.get()
        # print('expanding:', current.coordinates)
        # print('neighbours:', heuristic_graph.get_neighbours_of(current))
        expanded_nodes.add(current)

        # look at the distances to all neighbours
        # TODO optimisation: order neighbouring nodes after heuristic, keep multiple generators open
        # then do not check all neighbours first, but always the one with lowest heuristic
        for next_node, distance in heuristic_graph._existing_edges_from(current):
            if next_node in expanded_nodes:
                # this node has already been visited. there is no need to consider costs
                # path can only get longer by visiting other nodes first: new_cost is never < cost_so_far
                continue

            # print('visiting:', next_node.coordinates)
            if next_node == goal:
                # since the current node is the one with the lowest cost estimate
                #   and the goal is directly reachable from the current node (-> heuristic == distance),
                # the cost estimate is actually the true cost.
                # because of the geometric property mentioned above there can be no other shortest path to the goal
                # the algorithm can be terminated (regular a* would now still continue to all other neighbours first)
                # optimisation: _exiting_edges_from() returns the goal node first if it is among the neighbours
                # print('reached goal node. terminating')
                total_path_length = cost_so_far[current] + distance
                came_from[next_node] = current
                return reconstruct_path(), total_path_length

            new_cost = cost_so_far[current] + distance
            if new_cost < cost_so_far.get(next_node, infinity):
                # print('shortest path to this node so far:', new_cost)
                cost_so_far[next_node] = new_cost
                came_from[next_node] = current
                priority = new_cost + heuristic_graph.get_heuristic(next_node)
                priority_queue.put(next_node, priority)

    # goal is not reachable
    return [], None
