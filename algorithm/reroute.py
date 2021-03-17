import os
import sys
import math

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci

def rerouter(start, end, batteryCapacity, graph):
    startNode = graph.Net.getEdge(start).getFromNode().getID()
    endNode = graph.Net.getEdge(end).getToNode().getID()
    evRange = estimateRange(batteryCapacity)
    route = []
    routeLength = []
    csStops = []

    print('evRange: ', evRange)

    while True:
        tempRoute, tempLength = aStarSearch(graph, startNode, endNode)
        evRange = calculateCSRefuel(evRange, csStops, tempLength)

        print("tempLength: ", tempLength)

        if evRange > tempLength[-1]:
            route += tempRoute
            routeLength += tempLength
            break

        tempRoute, tempLength, csStop = routeViaCS(graph, startNode, endNode, evRange)

        if tempRoute == None:
            break

        route += tempRoute
        routeLength += tempLength
        startNode = graph.Net.getEdge(route[-1]).getToNode().getID()

        # Get distance startNode to CS, to get current EV range
        evRange = evRange - (routeLength[-2] - csStop.StartPos)
        csStops.append(csStop)

    print(route)
    print(csStops)
    return route, csStops

def routeViaCS(graph, startNode, endNode, evRange):
    closestCSs = getNeighbouringCS(graph, startNode, endNode, evRange)

    if len(closestCSs) > 0:
        chargingStation = getBestCS(graph, closestCSs)
        csStartNode = graph.Net.getEdge(chargingStation.Lane).getFromNode().getID()
        csEndNode = graph.Net.getEdge(chargingStation.Lane).getToNode().getID()

        route, routeLength = aStarSearch(graph, startNode, csStartNode)

        routeLength.append(routeLength[-1] + graph.Net.getEdge(chargingStation.Lane).getLength())
        route.append(chargingStation.Lane)

        return route, routeLength, chargingStation

    return None

def getBestCS(graph, closestCSs):
    for cs in closestCSs:
        cs.VehiclesCharging = traci.chargingstation.getVehicleCount(cs.id)
        csChargePerStep = (cs.Power * cs.Efficiency) / 3600

    return closestCSs[0]

# Get the correct range and duration needed from and for EV
def calculateCSRefuel(evRange, csStops, routeLength):
    if len(csStops) > 0:
        chargingStation = csStops[-1]

        csChargePerStep = (chargingStation.Power * chargingStation.Efficiency) / 3600
        print('csChargePerStep', csChargePerStep)
        evRange = estimateRange(1000)

    return evRange

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
            if currentNode == None or routeCost[node] + distanceBetweenNodes(graph, node, end) < routeCost[currentNode] + distanceBetweenNodes(graph, currentNode, end):
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
# Used for heuristic in A*
def distanceBetweenNodes(graph, currentNode, endNode):
    currentCoords = graph.Net.getNode(currentNode).getCoord()
    endCoords = graph.Net.getNode(endNode).getCoord()

    return euclideanDistance(currentCoords, endCoords)

# Calculates distance between to points from co-ords
def euclideanDistance(aCoords, bCoords):
    x = bCoords[0] - aCoords[0]
    y = bCoords[1] - aCoords[1]

    return math.sqrt((x ** 2) + (y ** 2))

# Converts the route to be in edges not nodes for sumo vehicle to follow
def reconstructRoutePath(graph, start, current, route):
    newRoute = []
    routeLength = []
    length = 0

    while route[current] != current:
        connectingEdge = graph.getNodeEdge(route[current], current)

        length += connectingEdge['Length']
        routeLength.append(length)
        newRoute.append(connectingEdge['ConnectingEdge'])
        current = route[current]

    newRoute.reverse()

    return newRoute, routeLength

# Estimates range for EV from current battery capacity
# http://www.ev-propulsion.com/EV-calculations.html
def estimateRange(batteryCapacity):
    return round((batteryCapacity / 180) * 1000, 2)

def getNeighbouringCS(graph, mainNode, endNode, radius):
    nodeCoords = graph.Net.getNode(mainNode).getCoord()
    endCoords = graph.Net.getNode(endNode).getCoord()
    lineDistance = euclideanDistance(nodeCoords, endCoords)
    chargingStations = []

    for cs in graph.ChargingStations:
        if checkCSInRadius(nodeCoords, cs.X, cs.Y, radius):
            cs.DistanceFromStart = euclideanDistance(nodeCoords, [cs.X, cs.Y])
            cs.DistanceFromDivider = distanceFromLine(nodeCoords, endCoords, [cs.X, cs.Y], lineDistance)

            chargingStations.append(cs)

    return chargingStations

# Use pythagoras to get distance between point and see if lower than the radius
def checkCSInRadius(nodeCoords, csX, csY, radius):
    distance = (nodeCoords[0] - csX) ** 2 + (nodeCoords[1] - csY) ** 2
    return distance <= radius ** 2

# Distance calculation from point to a line
# https://geomalgorithms.com/a02-_lines.html
def distanceFromLine(lineA, lineB, csCoords, lineDistance):
    eqTop = ((lineB[0] - lineA[0]) * (lineA[1] - csCoords[1])) - ((lineA[0] - csCoords[0]) * (lineB[1] - lineA[1]))

    return abs(eqTop / lineDistance)
