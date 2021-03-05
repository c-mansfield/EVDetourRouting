class Graph:
    def __init__(self, net):
        self.edges = net.getEdges()
        self.nodes = net.getNodes()
        self.nodeNeighbours = self.getNodeNeighbours(net)

    def neighbors(self, id):
        return self.nodeNeighbours.get(id)

    # Get the cost of going from # to node
    def getNodesConnectingEdge(self, fromNode, toNode):
        nodesNeighbours = self.nodeNeighbours.get(fromNode)
        return next(props['ConnectingEdge'] for props in nodesNeighbours if props['Neighbour'] == toNode)

    # Get the cost of going from # to node
    def cost(self, fromNode, toNode):
        nodesNeighbours = self.nodeNeighbours.get(fromNode)
        return [props['Length'] for props in nodesNeighbours if props['Neighbour'] == toNode]

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
