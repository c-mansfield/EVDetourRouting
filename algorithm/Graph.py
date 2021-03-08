class Graph:
    def __init__(self, net):
        self.edges = net.getEdges()
        self.nodes = net.getNodes()
        self.nodeNeighbours = self.getNodeNeighbours(net)

    def neighbors(self, id):
        return self.nodeNeighbours.get(id)

    # Get the edge and length between two nodes
    def getNodeEdge(self, fromNode, toNode):
        nodesNeighbours = self.nodeNeighbours.get(fromNode)
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
