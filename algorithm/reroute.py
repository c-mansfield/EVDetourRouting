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

def rerouter(start, end, batteryCapacity):
    graph = Graph(net)
    route, routeLength = aStarSearch(graph, net.getEdge(start).getFromNode().getID(), net.getEdge(end).getToNode().getID())

    print("routeLength ", routeLength)
    print('Distance: ', calculateRange(batteryCapacity))

    if calculateRange(batteryCapacity) > routeLength:
        return route

    return None

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

    return None, 0

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
    routeLength = 0

    while route[current] != current:
        connectingEdge = graph.getNodeEdge(route[current], current)

        routeLength += connectingEdge['Length']
        newRoute.append(connectingEdge['ConnectingEdge'])
        current = route[current]

    # newRoute.append(start)

    newRoute.reverse()

    return newRoute, routeLength

# Estimates range for EV
# http://www.ev-propulsion.com/EV-calculations.html
def calculateRange(batteryCapacity):
    return round((batteryCapacity / 330) * 1000, 2)
