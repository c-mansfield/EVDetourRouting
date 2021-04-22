import os
import sys
import xml.etree.ElementTree as ET
from algorithm.ChargingStation import ChargingStation

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib

class Graph:
    def __init__(self, netFile, additionalFile):
        self.Net = sumolib.net.readNet(netFile)
        self.Edges = self.Net.getEdges()
        self.Nodes = self.Net.getNodes()
        self.NodeNeighbours = self.getNodeNeighbours()
        self.ChargingStations = self.getChargingStations(additionalFile)
        self.MaxSpeed = self.getMaxSpeed(netFile)
        self.Connections = self.getConnections(netFile)

    def neighbors(self, id):
        return self.NodeNeighbours.get(id)

    # Get the edge and length between two nodes
    def getNodeEdge(self, fromNode, toNode):
        nodesNeighbours = self.NodeNeighbours.get(fromNode)
        return next(edge for edge in nodesNeighbours if edge['Neighbour'] == toNode)

    def getNodeNeighbours(self):
        netNodes = self.Net.getNodes()
        nodes = {}

        for i in netNodes:
            nodeConnections = []

            for j in self.Net.getNode(i.getID()).getOutgoing():
                # Make sure only gets lanes with evehicle access
                if j.getToNode().getID() and j.allows('evehicle'):
                    nodeConnections.append({
                        'Neighbour': j.getToNode().getID(),
                        'ConnectingEdge': j.getID(),
                        'Length': j.getLength()
                    })

                    nodes.update([(i.getID(), nodeConnections)])

        return nodes

    # Gets all charging stations and attributes in the sumo simulation net
    def getChargingStations(self, additionalFile):
        chargingStations = []
        xmlTree = ET.parse(additionalFile)
        root = xmlTree.getroot()

        for element in root.findall('chargingStation'):
            x, y = sumolib.geomhelper.positionAtShapeOffset(self.Net.getEdge(element.get('lane').split('_')[0]).getShape(), float(element.get('startPos')))
            cs = ChargingStation(element.get('id'), element.get('lane'), x, y, float(element.get('startPos')), float(element.get('endPos')), float(element.get('power')), float(element.get('efficiency')))

            chargingStations.append(cs)

        return chargingStations

    # Gets the max speed in the sumo simulation net
    def getMaxSpeed(self, netFile):
        xmlTree = ET.parse(netFile)
        root = xmlTree.getroot()
        maxSpeed = 0

        for lane in root.findall('.//edge/lane'):
            laneSpeed = float(lane.get('speed'))
            maxSpeed = max(laneSpeed, maxSpeed)

        return maxSpeed

    # Workaround function due to .getConnections broken in SUMO
    def getConnections(self, netFile):
        connections = {}
        xmlTree = ET.parse(netFile)
        root = xmlTree.getroot()
        internalLanes = sumolib.net.readNet(netFile, withInternal=True)

        for con in root.findall('connection'):
            if con.get('via') != None and internalLanes.getLane(con.get('via')).allows('evehicle'):
                connectionObj = {
                    'to': con.get('to'),
                    'via': con.get('via'),
                    'length': internalLanes.getLane(con.get('via')).getLength()
                }
                allConnections = connections.get(con.get('from'))

                if allConnections == None:
                    allConnections = []

                allConnections.append(connectionObj)
                connections.update([(con.get('from'), allConnections)])

        return connections
