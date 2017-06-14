import heapq
import warnings
from pextant.mesh.abstractcomponents import MeshCollection

class aStarSearchNode(object):
    def __init__(self, state, parent=None, cost_from_parent=0):
        self.state = state
        self.parent = parent
        self.cost_from_parent = cost_from_parent

    def goalTest(self, goal):
        return self.state == goal.state

    def getPath(self):
        node = self
        statepath = [node.state]
        nodepath = [node]
        while node.parent:
            statepath.append(node.parent.state)
            nodepath.append(node.parent)
            node = node.parent

        statepath.reverse()
        nodepath.reverse()
        return (statepath, nodepath)

    def getChildren(self):
        pass

class aStarNodeCollection(object):
    """
    class to be overwritten
    """
    def __init__(self, collection):
        '''
        :type collection: MeshCollection
        '''
        self.collection = collection

    def __getitem__(self, index):
        return aStarSearchNode(self.collection[index])

    #TODO: need to enforce this
    def get_states(self):
        return self.collection.get_states()

class aStarCostFunction(object):
    def __init__(self):
        self.end_node = None
        self.cache = False
        self.cached = {
            "costs": None,
            "heuristics": None
        }

    def setEndNode(self, end_state):
        elt = end_state.mesh_element
        if self.cache:
            self.cached["heuristics"] = self.cache_heuristic((elt.x, elt.y))
        self.end_node = elt

    def cache_heuristic(self, end_state):
        pass

    def getHeuristicCost(self, node):
        return 0

    def getCostBetween(self, fromnode, tonode):
        return 0

def aStarSearch(start_node, end_node, cost_function, viz=None):
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
        return (start_node, 0, 0)

    cost_function.setEndNode(end_node)
    start_node.cost = 0
    # agenda contains pairs (cost, node)
    agenda = []
    current_node = start_node # needed in case of early exit of loop
    heapq.heappush(agenda, (0, start_node))
    g_cost = {start_node.state: 0}
    expanded = set()

    while agenda:
        current_node = heapq.heappop(agenda)[1]
        current_node_state = current_node.state
        if current_node.goalTest(end_node):
            return (current_node.getPath(), expanded)
        expanded.add(current_node_state)
        children = current_node.getChildren()
        costs_to_node = g_cost[current_node_state] + cost_function.getCostBetween(current_node, children)
        children_states = children.get_states()
        for idx, child_node in enumerate(children):
            child_node_state = children_states[idx]
            cost_to_node = costs_to_node[idx]
            if child_node_state in expanded: #and cost_to_node >= g_cost.get(child_node_state,0):
                continue
            test = g_cost.get(child_node_state, float("inf"))
            if cost_to_node < test:
                # this if statement is a shortcut that does a lot of things at the same time
                g_cost[child_node_state] = cost_to_node
                estimated_cost = cost_to_node + cost_function.getHeuristicCost(child_node)
                heapq.heappush(agenda, (estimated_cost, child_node))
                if viz:
                    viz.add(child_node_state, estimated_cost)
        if viz:
            viz.addcount()

    # if it can't find a solution
    warnings.warn('no solution found')
    #if viz:
    #    viz.draw()
    return (([],[]), expanded)