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
    route = aStarSearch(graph, net.getEdge(start).getFromNode().getID(), net.getEdge(end).getToNode().getID())

    return route

def aStarSearch(graph, start, end):
    openList = set([start])
    closedList = set([])

    route = {}
    route[start] = start
    routeCost = {}
    routeCost[start] = 0

    while len(openList) > 0:
        currentNode = None

        for node in openList:
            if currentNode == None or routeCost[node] + heuristic(node, end) < routeCost[currentNode] + heuristic(currentNode, end):
                currentNode = node;

        if currentNode == None:
            return None

        if currentNode == end:
            return reconstructRoutePath(graph, start, currentNode, route)

        for next in graph.neighbors(currentNode):
            neighbourNode = next['Neighbour']

            if neighbourNode not in openList and neighbourNode not in closedList:
                openList.add(neighbourNode)
                route[neighbourNode] = currentNode
                routeCost[neighbourNode] = routeCost[currentNode] + next['Length']

            else:
                if routeCost[neighbourNode] > routeCost[currentNode] + next['Length']:
                    routeCost[neighbourNode] = routeCost[currentNode] + next['Length']
                    route[neighbourNode] = currentNode

                    if neighbourNode in closedList:
                        closedList.remove(neighbourNode)
                        openList.add(neighbourNode)

        closedList.add(currentNode)
        openList.remove(currentNode)

    return route

# Get distance from node to end node using euclidean distance
def heuristic(currentNode, endNode):
    currentCoords = net.getNode(currentNode).getCoord()
    endCoords = net.getNode(endNode).getCoord()

    x = currentCoords[0] - endCoords[0]
    y = endCoords[0] - endCoords[0]

    return ((x ** 2) + (y ** 2)) ** 0.5

# Converts the route to be in edges not nodes for sumo vehicle to follow
def reconstructRoutePath(graph, start, current, route):
    newRoute = []

    while route[current] != current:
        newRoute.append(graph.getNodesConnectingEdge(route[current], current))
        current = route[current]

    # newRoute.append(start)

    newRoute.reverse()

    return newRoute
