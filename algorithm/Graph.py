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
    def __init__(self, net, additionalFile):
        self.Net = net
        self.Edges = net.getEdges()
        self.Nodes = net.getNodes()
        self.NodeNeighbours = self.getNodeNeighbours(net)
        self.ChargingStations = self.getChargingStations(additionalFile)

    def neighbors(self, id):
        return self.NodeNeighbours.get(id)

    # Get the edge and length between two nodes
    def getNodeEdge(self, fromNode, toNode):
        nodesNeighbours = self.NodeNeighbours.get(fromNode)
        return next(edge for edge in nodesNeighbours if edge['Neighbour'] == toNode)

    def getNodeNeighbours(self, net):
        netNodes = net.getNodes()
        nodes = {}

        for i in netNodes:
            nodeConnections = []

            for j in net.getNode(i.getID()).getOutgoing():
                if j.getToNode().getID():
                    nodeConnections.append({
                        'Neighbour': j.getToNode().getID(),
                        'ConnectingEdge': j.getID(),
                        'Length': j.getLength()
                    })

            nodes.update([(i.getID(), nodeConnections)])

        return nodes

    def getChargingStations(self, additionalFile):
        chargingStations = []
        xmlTree = ET.parse(additionalFile)
        root = xmlTree.getroot()

        for element in root.findall('chargingStation'):
            x, y = sumolib.geomhelper.positionAtShapeOffset(self.Net.getEdge(element.get('lane').split('_')[0]).getShape(), float(element.get('startPos')))
            cs = ChargingStation(element.get('id'), element.get('lane'), x, y, float(element.get('startPos')), float(element.get('endPos')), float(element.get('power')), float(element.get('efficiency')))

            chargingStations.append(cs)

        return chargingStations
