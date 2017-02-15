import numpy as np
import numpy.matlib as matlib
import math
from astar import aStarSearchNode, aStarCostFunction

class SearchKernel(object):
    def __init__(self, kernelrange = 3):
        searchvector = range(-(kernelrange / 2), kernelrange / 2 + 1)
        col_off = matlib.repmat(searchvector, len(searchvector), 1)
        row_off = np.transpose(col_off)
        self.length = kernelrange**2
        self.col_off = col_off.flatten()
        self.row_off = row_off.flatten()

    def getKernel(self):
        return np.array([self.col_off, self.row_off])

class MeshElement(object):
    def __init__(self, row, col, parentMesh):
        self.row = row
        self.col = col
        self.parentMesh = parentMesh

    def getCoordinates(self):
        return np.array([self.row, self.col])

    def getElevation(self):
        return self.parentMesh.dataset[self.row, self.col]

    def getNeighbours(self):
        return self.parentMesh.getNeighbours(self)

    def __str__(self):
        return '(%s, %s)' % (self.row, self.col)

class MeshCollection(object):
    def __init__(self):
        self.collection = []
    def raw(self):
        return [(mesh_elt.row, mesh_elt.col) for mesh_elt in self.collection]
    def __str__(self):
        return str(self.raw())

class MeshSearchElement(aStarSearchNode):
    def __init__(self, mesh_element, parent=None):
        self.mesh_element = mesh_element
        state = (mesh_element.row, mesh_element.col)
        super(MeshSearchElement, self).__init__(state, parent, 0)

    def getChildren(self):
        return [MeshSearchElement(mesh_element, self)
                for mesh_element in self.mesh_element.getNeighbours().collection]

    def __str__(self):
        return str(self.mesh_element)

class astronautCost(aStarCostFunction):
    def __init__(self, astronaut, environment, optimize_on):
        super(astronautCost, self).__init__()
        self.explorer = astronaut
        self.map = environment
        self.optimize_vector = self._vectorize(optimize_on)
    """
    Given the start and end states, returns the cost of travelling between them.
    Allows for states which are not adjacent to each other.

    Note that this uses start and end coordinates rather than nodes.

    optimize_vector is a list or tuple of length 3, representing the weights of
    Distance, Time, and Energy
    """

    @staticmethod
    def _vectorize(optimize_on):
        if optimize_on == 'Energy':
            return [0, 0, 1]
        elif optimize_on == 'Time':
            return [0, 1, 0]
        elif optimize_on == 'Distance':
            return [1, 0, 0]
        elif isinstance(optimize_on, list) and len(optimize_on) == 3:
            return optimize_on
        else:
            raise TypeError(
                "optimize_on must be a list of length 3, \'Energy\', \'Time\', or \'Distance\', given" + repr(
                    optimize_on))

    def getHeuristicCost(self, node):
        start_row, start_col = node.state
        end_row, end_col = self.end_node.state

        optimize_vector = self.optimize_vector

        h_diagonal = min(abs(start_row - end_row),
                         abs(start_col - end_col))  # max number of diagonal steps that can be taken
        h_straight = abs(start_row - end_row) + abs(start_col - end_col)  # Manhattan distance

        # D represents the cost between two consecutive nodes
        d = 0 #TODO: need to clean up

        # Adding the energy weight
        m = self.explorer.mass
        r = self.map.resolution
        if self.map.planet == 'Earth':
            d += (1.504 * m + 53.298) * r * optimize_vector[2]
        elif self.map.planet == 'Moon' and self.explorer.type == 'Astronaut':
            d += (2.295 * m + 52.936) * r * optimize_vector[2]
        elif self.map.planet == 'Moon' and self.explorer.type == 'Rover':
            p_e = self.explorer.P_e  # this only exists for rovers
            d += (0.216 * m + p_e / 4.167) * r * optimize_vector[2]
        else:
            # This should not happen
            raise TypeError("planet/explorer conflict, current planet: ", self.map.planet, "current explorer: ",
                            self.explorer.type)

        # Adding the distance weight
        d += self.map.resolution * optimize_vector[0]

        # Adding the time weight
        max_velocity = 1.6  # the maximum velocity is 1.6 from Marquez 2008
        d += self.map.resolution * optimize_vector[1] / max_velocity

        heuristic_cost = 100  # a large number

        # Patel 2010. See page 49 of Aaron's thesis
        heuristic_cost = d * math.sqrt(2) * h_diagonal + d * (h_straight - 2 * h_diagonal)
        # This is just Euclidean distance
        #heuristic_cost = d * math.sqrt((start_row - end_row) ** 2 + (start_col - end_col) ** 2)

        return heuristic_cost

    def getActualCost(self, fromnode, tonode):
        optimize_vector = self.optimize_vector

        r = self.map.resolution
        start_row, start_col = fromnode.state
        end_row, end_col = tonode.state
        start_elev, end_elev = fromnode.mesh_element.getElevation(), tonode.mesh_element.getElevation()

        path_length = r * math.sqrt((start_row - end_row) ** 2 + (start_col - end_col) ** 2)
        slope = math.degrees(math.atan((end_elev - start_elev) / path_length))

        if optimize_vector[0]:
            dist_weight = self.explorer.distance(path_length) * optimize_vector[0]
        else:
            dist_weight = 0

        if optimize_vector[1]:
            time_weight = self.explorer.time(path_length, slope) * optimize_vector[1]
        else:
            time_weight = 0

        if optimize_vector[2]:
            energy_weight = self.explorer.energyCost(path_length, slope, self.map.getGravity()) * optimize_vector[2]
        else:
            energy_weight = 0

        return dist_weight + time_weight + energy_weight

if __name__ == '__main__':
    from loadWaypoints import loadPoints
    from EnvironmentalModel import GDALMesh
    from ExplorerModel import Astronaut
    from astar import aStarSearch
    hi_low = GDALMesh('maps/HI_lowqual_DEM.tif')
    waypoints = loadPoints('waypoints/HI_13Nov16_MD7_A.json')
    env_model = hi_low.loadMapSection(waypoints.geoEnvelope())
    astronaut = Astronaut(80)
    cost_function = astronautCost(astronaut, env_model, "Energy")
    # env_model.generateRelief(50)
    start_node = MeshSearchElement(env_model.getMeshElement(1, 1))
    end_node = MeshSearchElement(env_model.getMeshElement(10, 10))
    print aStarSearch(start_node, end_node, cost_function)