import os
import sys
import math
from random import shuffle

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci
from algorithm.Graph import Graph

def rerouter(start, end, evID, netFile, additionalFile):
    graph = Graph(netFile, additionalFile)

    startNode = graph.Net.getEdge(start).getFromNode().getID()
    endNode = graph.Net.getEdge(end).getToNode().getID()
    batteryCapacity = float(traci.vehicle.getParameter(evID, 'device.battery.actualBatteryCapacity'))
    evRange = estimateRange(evID, batteryCapacity)
    route = []
    routeLength = 0
    csStops = []

    print('evRange: ', evRange)
    print('batteryCapacity: ', batteryCapacity)

    while True:
        tempRoute, tempLength = aStarSearch(graph, startNode, endNode, evRange)

        if tempRoute != None:
            route += tempRoute
            routeLength += tempLength
            break

        tempRoute, tempLength, csStop = routeViaCS(graph, startNode, endNode, evRange)

        if tempRoute == None:
            print('No valid route for EV with current capacity')
            break

        route += tempRoute
        routeLength += tempLength
        startNode = graph.Net.getEdge(route[-1]).getToNode().getID()

        # Get current EV range that has just been travelled
        # lastEdgeLength = graph.Net.getEdge(route[-1]).getLength()
        evRange -= tempLength
        evRange, csStop = calculateCSRefuel(evRange, csStop, tempLength, evID)
        csStops.append(csStop)

    print('route: ', route)
    print('routeLength: ', routeLength)
    print('csStops: ', csStops)
    return route, csStops

def routeViaCS(graph, startNode, endNode, evRange):
    closestCSs = getNeighbouringCS(graph, startNode, endNode, evRange)

    if len(closestCSs) > 0:
        chargingStation = getBestCS(graph, closestCSs)
        csStartNode = graph.Net.getEdge(chargingStation.Lane).getFromNode().getID()
        csEndNode = graph.Net.getEdge(chargingStation.Lane).getToNode().getID()

        route, routeLength = aStarSearch(graph, startNode, csStartNode, evRange)

        if route == None:
            return None, None, None

        # Append the connecting node to the edge where the CS
        # lies incase more than one edge coming from start node
        routeLength += graph.Net.getEdge(chargingStation.Lane).getLength()
        route.append(chargingStation.Lane)

        return route, routeLength, chargingStation

    return None, None, None

def getBestCS(graph, closestCSs):
    closestCSs.sort(key=lambda x: (x.DistanceFromStart, x.DistanceFromDivider))
    closestCSs = closestCSs[:10]

    for cs in closestCSs:
        cs.VehiclesCharging = traci.chargingstation.getVehicleCount(cs.id)
        csChargePerStep = (cs.Power * cs.Efficiency) / 3600

    return closestCSs[0]

# Get the correct range and duration needed from and for EV
def calculateCSRefuel(evRange, chargingStation, routeLength, evID):
    # Get meters still needed to travel to complete journey
    rangeNeeded = routeLength - evRange
    capacityNeeded = estimateBatteryCapacity(evID, rangeNeeded) * 1.2

    # Preference from user what percentage capacity they would like after charging
    capacityGoal = float(traci.vehicle.getParameter(evID, 'device.battery.maximumBatteryCapacity')) * 0.5

    csChargePerStep = (chargingStation.Power * chargingStation.Efficiency) / 3600
    durationToNeeded = math.ceil(capacityNeeded / csChargePerStep)
    durationToGoal = math.ceil(capacityGoal / csChargePerStep)

    # Get higher duration of two for time spent charging at CS
    chargingStation.Duration = max(durationToNeeded, durationToGoal)
    newCapacity = evRange + (chargingStation.Duration * csChargePerStep)

    evRange = estimateRange(evID, newCapacity)

    return evRange, chargingStation

def aStarSearch(graph, start, end, evRange):
    openList = set([start])
    closedList = set([])

    route = {}
    route[start] = start

    routeCost = {}
    routeCost[start] = 0

    routeLength = {}
    routeLength[start] = 0

    while len(openList) > 0:
        currentNode = None

        for node in openList:
            if currentNode == None \
                or routeCost[node] + heuristic(graph, node, end) < routeCost[currentNode] + heuristic(graph, currentNode, end):
                currentNode = node;

        if currentNode == None:
            return None

        if currentNode == end:
            return reconstructRoutePath(graph, start, currentNode, route, routeLength)

        for next in graph.neighbors(currentNode):
            neighbourNode = next['Neighbour']
            edgeStepSpeed = traci.edge.getLastStepMeanSpeed(next['ConnectingEdge'])

            if neighbourNode not in openList and neighbourNode not in closedList:
                openList.add(neighbourNode)
                route[neighbourNode] = currentNode

                # Travel time is cost of each node, length / speed of road, this gets fastest and shortest route
                routeCost[neighbourNode] = routeCost[currentNode] + (next['Length'] / edgeStepSpeed)
                routeLength[neighbourNode] = routeLength[currentNode] + next['Length']

            else:
                if routeCost[neighbourNode] > routeCost[currentNode] + (next['Length'] / edgeStepSpeed):
                    routeCost[neighbourNode] = routeCost[currentNode] + (next['Length'] / edgeStepSpeed)
                    route[neighbourNode] = currentNode
                    routeLength[neighbourNode] = routeLength[currentNode] + next['Length']

                    if neighbourNode in closedList:
                        closedList.remove(neighbourNode)
                        openList.add(neighbourNode)

        if evRange < list(routeLength.values())[-1]:
            print('Error, cannot find valid route with current range. Reroute via CS')
            break

        closedList.add(currentNode)
        openList.remove(currentNode)

    return None, 0

# Estimating the heristic as the euclidean distance from current to end divided
# by the max speed of any
def heuristic(graph, currentNode, endNode):
    return distanceBetweenNodes(graph, currentNode, endNode) / 13.049

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
def reconstructRoutePath(graph, start, current, route, routeLength):
    newRoute = []

    while route[current] != current:
        connectingEdge = graph.getNodeEdge(route[current], current)

        newRoute.append(connectingEdge['ConnectingEdge'])
        current = route[current]

    newRoute.reverse()

    return newRoute, list(routeLength.values())[-1]

# Estimates range for EV from current battery capacity
# Returns value in meters
# https://sumo.dlr.de/docs/Models/Electric.html#calculating_the_remaining_range
def estimateRange(evID, batteryCapacity):
    return batteryCapacity * getMetersPerWatt(evID)

# Estimate the battery capacity needed from refuel at charging station to get to destination
# Returns value in Wh
def estimateBatteryCapacity(evID, evRange):
    return evRange / getMetersPerWatt(evID)

# Gte the meters per Watt-hour of the current EV to use in range and capacity calculations
def getMetersPerWatt(evID):
    # mWh = traci.vehicle.getDistance(evID) / float(traci.vehicle.getElectricityConsumption(evID))
    mWh = 4.665999805641006
    return mWh

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
