import os
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib

def rerouter(start, end, batteryCapacity, graph):
    startNode = graph.Net.getEdge(start).getFromNode().getID()
    endNode = graph.Net.getEdge(end).getToNode().getID()
    evRange = calculateRange(batteryCapacity)

    route, routeLength = aStarSearch(graph, startNode, endNode, evRange)

    print("routeLength ", routeLength)
    print('EV Range: ', calculateRange(batteryCapacity))

    if evRange > routeLength[len(routeLength)-1]:
        return route

    return None

def aStarSearch(graph, start, end, evRange):
    openList = set([start])
    closedList = set([])

    route = {}
    route[start] = start
    routeCost = {}
    routeCost[start] = 0

    while len(openList) > 0:
        currentNode = None

        for node in openList:
            if currentNode == None or routeCost[node] + heuristic(graph, node, end) < routeCost[currentNode] + heuristic(graph, currentNode, end):
                currentNode = node;

        if currentNode == None:
            return None

        if currentNode == end:
            return reconstructRoutePath(graph, start, currentNode, route, evRange)

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
def heuristic(graph, currentNode, endNode):
    currentCoords = graph.Net.getNode(currentNode).getCoord()
    endCoords = graph.Net.getNode(endNode).getCoord()

    x = currentCoords[0] - endCoords[0]
    y = endCoords[0] - endCoords[0]

    return ((x ** 2) + (y ** 2)) ** 0.5

# Converts the route to be in edges not nodes for sumo vehicle to follow
def reconstructRoutePath(graph, start, current, route, evRange):
    newRoute = []
    routeLength = []
    length = 0

    while route[current] != current:
        connectingEdge = graph.getNodeEdge(route[current], current)

        getNeighbouringCS(graph, current, 100)
        length += connectingEdge['Length']
        routeLength.append(length)
        newRoute.append(connectingEdge['ConnectingEdge'])
        current = route[current]

    newRoute.reverse()

    return newRoute, routeLength

# Estimates range for EV
# http://www.ev-propulsion.com/EV-calculations.html
def calculateRange(batteryCapacity):
    return round((batteryCapacity / 330) * 1000, 2)

def getNeighbouringCS(graph, mainNode, radius):
    nodeCoords = graph.Net.getNode(mainNode).getCoord().strip('()')
    nodeX, nodeY = nodeCoords()

    print(nodeCoords)
    # return result = [cs
    #                     for cs in graph.ChargingStations
    #                         if cs.X > ]
