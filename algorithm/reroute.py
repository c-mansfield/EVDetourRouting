from algorithm.PriorityQueue import PriorityQueue
from algorithm.Graph import Graph
import os
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib

net = sumolib.net.readNet('data/EVGrid.net.xml')

def rerouter(start, end):
    graph = Graph(net)
    route = a_star_search(graph, net.getEdge(start).getFromNode().getID(), net.getEdge(end).getToNode().getID())

    print(route)
    return route

def a_star_search(graph, start, end):
    openQueue = PriorityQueue()
    openQueue.put(start, 0)
    route = []
    routeCost = {}
    route.append(start)
    routeCost[start] = 0

    while not openQueue.empty():
        current = openQueue.get()

        if current == end:
            return route

        for next in graph.neighbors(current):
            newCost = routeCost[current] + graph.cost(current, next)

            if next not in routeCost.keys() or newCost < routeCost[next]:
                routeCost[next] = newCost
                totalCost = newCost + heuristic(next, end)
                openQueue.put(next, totalCost)
                if current not in route:
                    route.append(current)

    return route

# Get distance from node to end node using euclidean distance
def heuristic(currentNode, endNode):
    currentCoords = net.getNode(currentNode).getCoord()
    endCoords = net.getNode(endNode).getCoord()

    x = currentCoords[0] - endCoords[0]
    y = endCoords[0] - endCoords[0]

    return ((x ** 2) + (y ** 2)) ** 0.5
