from search.algorithms import State
from search.map import Map
import heapq
import sys

def Bi_Astar(start:State, goal:State, map:Map):

    openF = []
    openB = []
    closedF = {}
    closedB = {}
    nodes_expanded = 0

    delta_x = abs(start.get_x() - goal.get_x())
    delta_y = abs(start.get_y() - goal.get_y())

    h = 1.5 * min(delta_x, delta_y) + abs(delta_x - delta_y)
    f = start.get_g() + h
    start.set_cost(f)

    f = goal.get_g() + h
    goal.set_cost(f)

    heapq.heappush(openF, start)
    heapq.heappush(openB, goal)

    u = sys.maxsize

    while len(openF) > 0 and len(openB) > 0:
        if u <= openF[0].get_cost() and u <= openB[0].get_cost():
            return u, nodes_expanded
        if openF[0].get_cost() <= openB[0].get_cost():
            node = heapq.heappop(openF)
            nodes_expanded += 1
            for childNode in map.successors(node):

                delta_x = abs(childNode.get_x() - goal.get_x())
                delta_y = abs(childNode.get_y() - goal.get_y())
                h = 1.5 * min(delta_x, delta_y) + abs(delta_x - delta_y)
                f = childNode.get_g() + h
                childNode.set_cost(f)

                if childNode.state_hash() in closedB:
                    u = min(u, childNode.get_g() + closedB[childNode.state_hash()].get_g())
                if childNode.state_hash() not in closedF:
                    heapq.heappush(openF, childNode)
                    closedF[childNode.state_hash()] = childNode
                if childNode.state_hash() in closedF and childNode.get_cost() < closedF[childNode.state_hash()].get_cost():
                    heapq.heappush(openF, childNode)
                    closedF[childNode.state_hash()] = childNode
                    heapq.heapify(openF)
        else:
            node = heapq.heappop(openB)
            nodes_expanded += 1
            for childNode in map.successors(node):

                delta_x = abs(childNode.get_x() - start.get_x())
                delta_y = abs(childNode.get_y() - start.get_y())
                h = 1.5 * min(delta_x, delta_y) + abs(delta_x - delta_y)
                f = childNode.get_g() + h
                childNode.set_cost(f)

                if childNode.state_hash() in closedF:
                    u = min(u, childNode.get_g() + closedF[childNode.state_hash()].get_g())
                if childNode.state_hash() not in closedB:
                    heapq.heappush(openB, childNode)
                    closedB[childNode.state_hash()] = childNode
                if childNode.state_hash() in closedB and childNode.get_cost() < closedB[childNode.state_hash()].get_cost():
                    heapq.heappush(openB, childNode)
                    closedB[childNode.state_hash()] = childNode
                    heapq.heapify(openB)
                    
    return -1, nodes_expanded

