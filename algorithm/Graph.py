class Graph:
    def __init__(self, net):
        self.edges = net.getEdges()
        self.nodes = net.getNodes()
        self.nodeNeighbours = self.getNodeNeighbours(net)
        self.edgeNeighbours = self.getEdgeNeighbours(net)

    def neighbors(self, id):
        return self.nodeNeighbours.get(id)

    # Get the cost of going from # to node
    def cost(self, fromNode, toNode):
        return self.nodeNeighbours.get(fromNode, {}).get(toNode)

    def getNodeNeighbours(self, net):
        netNodes = net.getNodes()
        nodes = {}

        for i in netNodes:
            nodeConnections = {}

            for j in net.getNode(i.getID()).getOutgoing():
                if j.getToNode().getID():
                    nodeConnections.update([(j.getToNode().getID(), j.getLength())])

            nodes.update([(i.getID(), nodeConnections)])

        return nodes

    def getEdgeNeighbours(self, net):
        netEdges = net.getEdges()
        edges = {}

        for i in netEdges:
            connectedEdges = list()

            for j in net.getEdge(i.getID()).getOutgoing():
                if j.getID():
                    connectedEdges.append(j.getID())

            edges.update([(i.getID(),connectedEdges)])

        return edges
