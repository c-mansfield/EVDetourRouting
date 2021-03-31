import os
import sys
import math
from random import shuffle, uniform

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
        tempRoute, tempLength = aStarSearch(graph, startNode, endNode, evRange, evID, False)

        # When intitial route gets something back, saves route and sets new start as last node
        if tempRoute != None and tempRoute != []:
            route += tempRoute
            routeLength += tempLength
            startNode = graph.Net.getEdge(tempRoute[-1]).getToNode().getID()

            # End cycle for route search if reached the end
            if graph.Net.getEdge(tempRoute[-1]).getToNode().getID() == endNode:
                break

        tempRoute, tempLength, csStop = routeViaCS(graph, startNode, endNode, evRange, evID)

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

def routeViaCS(graph, startNode, endNode, evRange, evID):
    closestCSs = getNeighbouringCS(graph, startNode, endNode, evRange)

    if len(closestCSs) > 0:
        chargingStation = getBestCS(graph, closestCSs)
        csStartNode = graph.Net.getEdge(chargingStation.Lane).getFromNode().getID()
        csEndNode = graph.Net.getEdge(chargingStation.Lane).getToNode().getID()

        route, routeLength = aStarSearch(graph, startNode, csStartNode, evRange, evID, True)

        if route == None:
            return None, None, None

        # Append the connecting node to the edge where the CS
        # lies incase more than one edge coming from start node
        routeLength += graph.Net.getEdge(chargingStation.Lane).getLength()
        route.append(chargingStation.Lane)

        return route, routeLength, chargingStation

    return None, None, None

# Make MCDM based on SAW technique and normalizing the data using vector normalization
# https://hal.inria.fr/hal-01438251/document#:~:text=They%20used%20a%20ranking%20consistency,SAW%20is%20the%20vector%20normalization.
# http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.134.8891&rep=rep1&type=pdf
def getBestCS(graph, closestCSs):
    # Get vector lengths for denomatonator for all attributes
    lenDistStart = math.sqrt(sum(cs.DistanceFromStart ** 2 for cs in closestCSs))
    lenDistDivider = math.sqrt(sum(cs.DistanceFromDivider ** 2 for cs in closestCSs))
    lenPrice = math.sqrt(sum(cs.Price ** 2 for cs in closestCSs))
    lenVehiclesCharging = math.sqrt(sum(cs.VehiclesCharging ** 2 for cs in closestCSs))
    lenStepCharge = math.sqrt(sum(cs.ChargePerStep ** 2 for cs in closestCSs))

    for cs in closestCSs:
        # Normalize each attribute of CS wish to make decision based on and  its weighting
        distStartScore = (1 - catchZeroDivision(cs.DistanceFromStart, lenDistStart)) * 0.25
        distDividerScore = (1 - catchZeroDivision(cs.DistanceFromDivider, lenDistDivider)) * 0.25
        priceScore = (1 - catchZeroDivision(cs.Price, lenPrice)) * 0.05
        vehiclesChargingScore = (1 - catchZeroDivision(cs.VehiclesCharging, lenVehiclesCharging)) * 0.20
        stepChargeScore = catchZeroDivision(cs.ChargePerStep, lenStepCharge) * 0.25

        # Get CS score of best charging station
        cs.Score = distStartScore + distDividerScore + priceScore + stepChargeScore # + vehiclesChargingScore

        print('Charging Station: ', cs.id)
        print('Score: ', cs.Score)

    return max(closestCSs, key=lambda item: item.Score)

def catchZeroDivision(x, y):
    try:
        return x / y
    except ZeroDivisionError:
        return 0

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

def aStarSearch(graph, start, end, evRange, evID, csRouting):
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

        # Checks whether soc under limit when getting intial route or route from CS
        if not csRouting:
            currentSOC = estimateSOC(evID, evRange, list(routeLength.values())[-1])

            if currentSOC < 11:
                return reconstructRoutePath(graph, start, currentNode, route, routeLength)

        for next in graph.neighbors(currentNode):
            neighbourNode = next['Neighbour']
            edgeStepSpeed = traci.edge.getLastStepMeanSpeed(next['ConnectingEdge'])

            if neighbourNode not in openList and neighbourNode not in closedList:
                openList.add(neighbourNode)
                route[neighbourNode] = currentNode

                # Travel time is cost of each node, length / speed of road, this gets fastest and shortest route
                routeCost[neighbourNode] = routeCost[currentNode] + catchZeroDivision(next['Length'], edgeStepSpeed)
                routeLength[neighbourNode] = routeLength[currentNode] + next['Length']

            else:
                if routeCost[neighbourNode] > routeCost[currentNode] + catchZeroDivision(next['Length'], edgeStepSpeed):
                    routeCost[neighbourNode] = routeCost[currentNode] + catchZeroDivision(next['Length'], edgeStepSpeed)
                    route[neighbourNode] = currentNode
                    routeLength[neighbourNode] = routeLength[currentNode] + next['Length']

                    if neighbourNode in closedList:
                        closedList.remove(neighbourNode)
                        openList.add(neighbourNode)

        # print('evRange: ', evRange)
        # print('list(routeLength.values())[-1]: ', list(routeLength.values())[-1])
        #
        # if evRange < list(routeLength.values())[-1]:
        #     print('Error, cannot find valid route with current range. Reroute via CS')
        #     break

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

# Get estimated state of charge for current spot in location
def estimateSOC(evID, evRange, routeLength):
    currentRange = evRange - routeLength
    return (estimateBatteryCapacity(evID, currentRange) / float(traci.vehicle.getParameter(evID, 'device.battery.maximumBatteryCapacity'))) * 100

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
            cs.VehiclesCharging = traci.chargingstation.getVehicleCount(cs.id)
            cs.Price = uniform(0.1, 0.25)

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
