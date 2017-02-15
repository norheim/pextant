import heapq

class aStarSearchNode(object):
    def __init__(self, state, parent, cost=0):
        self.state = state
        self.parent = parent
        self.cost = cost

    def goalTest(self, goal):
        return self.state == goal.state

    def getPath(self):
        path = [self.state]
        node = self
        while node.parent:
            path.append(node.parent.state)
            node = node.parent

        path.reverse()
        return path

    def getChildren(self):
        pass

class aStarCostFunction(object):
    def __init__(self):
        self.end_node = None

    def setEndNode(self, end_state):
        self.end_node = end_state

    def getHeuristicCost(self, node):
        return 0

    def getActualCost(self, fromnode, tonode):
        return 0

    def getEstimatedCost(self, fromnode, tonode):
        actual_cost = self.getActualCost(fromnode, tonode)
        tonode.cost = actual_cost
        heuristic_cost = self.getHeuristicCost(tonode)
        estimated_cost = actual_cost + heuristic_cost
        return estimated_cost

def aStarSearch(start_node, end_node, cost_function):
    """
    returns the path (as a list of coordinates), followed by the number of
    states expanded, followed by the total cost

    this function will return a list of states between two waypoints
    to get the full path between all waypoints, we will basically just use
    this function multiple times with each set
    as long as the number of waypoints is reasonable we should be fine

    costFunction supports a vector costFunction, with the
    three vector elements representing: 'Energy', 'Time', or 'Distance'
    As of right now 'Energy' just refers to metabolic energy.
    """
    if start_node.goalTest(end_node):
        return start_node.getPath()

    cost_function.setEndNode(end_node)
    start_node.cost = 0
    # agenda contains pairs (cost, node)
    agenda = []
    heapq.heappush(agenda, (0, start_node))
    expanded = set()
    while len(agenda) > 0:
        priority, node = heapq.heappop(agenda)
        if node.state not in expanded:
            expanded.add(node.state)
            if node.goalTest(end_node):
                return (node.getPath(), len(expanded), node.cost)
            for child_node in node.getChildren():
                if child_node.state not in expanded:
                    # calculate cost
                    estimated_cost = cost_function.getEstimatedCost(node, child_node)
                    heapq.heappush(agenda, (estimated_cost, child_node))

    return (None, len(expanded), node.cost)