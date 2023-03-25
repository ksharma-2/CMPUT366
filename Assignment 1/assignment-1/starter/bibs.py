import heapq
import sys
from search.algorithms import State
from search.map import Map

def BiBs(start: State, goal: State, map:Map):

    openF = []
    openB = []
    closedF = {}
    closedB = {}
    nodes_expanded = 0

    heapq.heappush(openF, start)
    heapq.heappush(openB, goal)

    closedF[start.state_hash()] = start
    closedB[goal.state_hash()] = goal

    u = sys.maxsize

    while len(openF) > 0 and len(openB) > 0:
        if u < openF[0].get_g() + openB[0].get_g():
            return u, nodes_expanded
        if openF[0].get_g() < openB[0].get_g():
            node = heapq.heappop(openF)
            nodes_expanded += 1
            for childNode in map.successors(node):
                if childNode.state_hash() in closedB:
                    u = min(u, childNode.get_g() + closedB[childNode.state_hash()].get_g())
                if childNode.state_hash() not in closedF:
                    heapq.heappush(openF, childNode)
                    closedF[childNode.state_hash()] = childNode
                if childNode.state_hash() in closedF and childNode.get_g() < closedF[childNode.state_hash()].get_g():
                    heapq.heappush(openF, childNode)
                    closedF[childNode.state_hash()] = childNode
                    heapq.heapify(openF)
        else:
            node = heapq.heappop(openB)
            nodes_expanded += 1
            for childNode in map.successors(node):
                if childNode.state_hash() in closedF:
                    u = min(u, childNode.get_g() + closedF[childNode.state_hash()].get_g())
                if childNode.state_hash() not in closedB:
                    heapq.heappush(openB, childNode)
                    closedB[childNode.state_hash()] = childNode
                if childNode.state_hash() in closedB and childNode.get_g() < closedB[childNode.state_hash()].get_g():
                    heapq.heappush(openB, childNode)
                    closedB[childNode.state_hash()] = childNode
                    heapq.heapify(openB)
    return -1, nodes_expanded