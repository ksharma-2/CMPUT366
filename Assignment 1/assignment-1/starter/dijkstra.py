import heapq
from search.map import Map
from search.algorithms import State

def dijktra(start: State, goal: State, map: Map):

    open = []
    closed = {}
    nodes_expanded = 0

    heapq.heappush(open, start)
    closed[start.state_hash] = start
    y = start.state_hash()

    while bool(open):
        node = heapq.heappop(open)
        nodes_expanded += 1
        if node == goal:
            return closed[goal.state_hash()].get_g(), nodes_expanded
        for childNode in map.successors(node):
            if childNode.state_hash() not in closed:
                heapq.heappush(open, childNode)
                closed[childNode.state_hash()] = childNode
            if (childNode.state_hash() in closed) and (childNode.get_g() < closed[childNode.state_hash()].get_g()):
                heapq.heappush(open, childNode)
                closed[childNode.state_hash()] = childNode
                heapq.heapify(open)

    return -1, nodes_expanded