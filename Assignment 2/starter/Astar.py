from search.algorithms import State
from search.map import Map
import heapq

def Astar(start:State, goal:State, map:Map, path):

    open = []
    closed = {}
    nodes_expanded = 0

    delta_x = abs(start.get_x() - goal.get_x())
    delta_y = abs(start.get_y() - goal.get_y())

    h = 1.5 * min(delta_x, delta_y) + abs(delta_x - delta_y)
    f = start.get_g() + h

    start.set_cost(f)
    goal.set_cost(0)

    heapq.heappush(open, start)
    closed[start.state_hash()] = start

    while bool(open):
        node = heapq.heappop(open)
        nodes_expanded += 1
        if node == goal:
            map.plot_map(closed, start, goal, path)
            return closed[goal.state_hash()].get_cost(), nodes_expanded
        for childNode in map.successors(node):
            delta_x = abs(childNode.get_x() - goal.get_x())
            delta_y = abs(childNode.get_y() - goal.get_y())
            h = 1.5 * min(delta_x, delta_y) + abs(delta_x - delta_y)
            f = childNode.get_g() + h
            childNode.set_cost(f)

            if childNode.state_hash() not in closed:
                heapq.heappush(open, childNode)
                closed[childNode.state_hash()] = childNode
            if (childNode.state_hash() in closed and childNode.get_cost() < closed[childNode.state_hash()].get_cost()):
                heapq.heappush(open, childNode)
                closed[childNode.state_hash()] = childNode
                heapq.heapify(open)

    
    return -1, nodes_expanded

