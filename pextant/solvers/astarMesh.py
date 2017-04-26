import math
import numpy as np
from SEXTANTsolver import sextantSearch, SEXTANTSolver
from astar import aStarSearchNode, aStarNodeCollection, aStarCostFunction, aStarSearch
from pextant.EnvironmentalModel import EnvironmentalModel
from pextant.lib.geoshapely import GeoPoint, GeoPolygon, LONG_LAT

class MeshSearchElement(aStarSearchNode):
    def __init__(self, mesh_element, parent=None, cost_from_parent=0):
        self.mesh_element = mesh_element
        self.derived = {} #the point of this is to store in memory expensive calculations we might need later
        super(MeshSearchElement, self).__init__(mesh_element.state, parent, cost_from_parent)

    def goalTest(self, goal):
        if self.mesh_element.distanceToElt(goal.mesh_element) < self.mesh_element.parentMesh.resolution*3:
            return True

    def getChildren(self):
        return MeshSearchCollection(self.mesh_element.getNeighbours(), self)

    def __str__(self):
        return str(self.mesh_element)

class MeshSearchCollection(aStarNodeCollection):
    def __init__(self, collection, parent=None):
        super(MeshSearchCollection, self).__init__(collection)
        self.derived = None
        self.parent = parent

    def __getitem__(self, index):
        mesh_search_element = MeshSearchElement(self.collection.__getitem__(index), self.parent)
        mesh_search_element.derived = dict(zip(['pathlength','time','energy'],self.derived[:,index]))
        return mesh_search_element

class ExplorerCost(aStarCostFunction):
    def __init__(self, astronaut, environment, optimize_on, heuristic_accelerate=1):
        """

        :param astronaut:
        :param environment:
        :type environment: EnvironmentalModel
        :param optimize_on:
        """
        super(ExplorerCost, self).__init__()
        self.explorer = astronaut
        self.map = environment
        self.optimize_vector = astronaut.optimizevector(optimize_on)
        self.heuristic_accelerate = heuristic_accelerate

    def getHeuristicCost(self, elt):
        node = elt.mesh_element
        start_row, start_col = node.y, node.x
        end_row, end_col = self.end_node.y, self.end_node.x
        optimize_vector = self.optimize_vector

        # max number of diagonal steps that can be taken
        h_diagonal = min(abs(start_row - end_row), abs(start_col - end_col))
        h_straight = abs(start_row - end_row) + abs(start_col - end_col)  # Manhattan distance

        # D represents the cost between two consecutive nodes
        d = 0 #TODO: need to clean up

        # Adding the distance weight
        d += self.map.resolution * optimize_vector[0]

        # Adding the time weight
        max_velocity = 1.6  # the maximum velocity is 1.6 from Marquez 2008
        d += self.map.resolution / max_velocity * optimize_vector[1]

        # Adding the energy weight
        m = self.explorer.mass
        r = self.map.resolution

        # Aaron's thesis page 50
        if self.map.planet == 'Earth':
            energy_weight = 1.504 * m + 53.298
        elif self.map.planet == 'Moon' and self.explorer.type == 'Astronaut':
            energy_weight = 2.295 * m + 52.936
        elif self.map.planet == 'Moon' and self.explorer.type == 'Rover':
            p_e = self.explorer.P_e  # this only exists for rovers
            energy_weight = (0.216 * m + p_e / 4.167)
        else:
            # This should not happen
            raise TypeError("planet/explorer conflict, current planet: ", self.map.planet, "current explorer: ",
                            self.explorer.type)
        d += energy_weight *  r * optimize_vector[2]

        # Patel 2010. See page 49 of Aaron's thesis
        heuristic_weight = self.heuristic_accelerate
        heuristic_cost = heuristic_weight*(d * math.sqrt(2) * h_diagonal + d * (h_straight - 2 * h_diagonal))
        # This is just Euclidean distance
        #heuristic_cost = d * math.sqrt((start_row - end_row) ** 2 + (start_col - end_col) ** 2)

        return heuristic_cost

    def getCostBetween(self, fromnode, tonodes):
        optimize_weights = self.optimize_vector
        optimize_vector = self.calculateCostBetween(fromnode.mesh_element, tonodes.collection)
        costs = np.dot(optimize_vector.transpose(), optimize_weights)
        tonodes.derived = optimize_vector

        return costs

    def calculateCostBetween(self, from_elt, to_elts):
        """
            Given the start and end states, returns the cost of travelling between them.
            Allows for states which are not adjacent to each other.

            optimize_vector is a list or tuple of length 3, representing the weights of
            Distance, Time, and Energy
            Performance optimization: tonodes instead of tonode, potentially numpy optimized, only need to load info
            from fromnode once
        """
        explorer  = self.explorer
        slopes, path_lengths = from_elt.slopeTo(to_elts)
        times = explorer.time(path_lengths, slopes)
        g = self.map.getGravity()
        energy_rate = explorer.energyRate(path_lengths, slopes, g)
        #TODO: rewrite this so not all functions need to get evaluated(expensive)
        optimize_vector = np.array([
            path_lengths,
            times,
            energy_rate*times
        ])
        return optimize_vector


class astarSolver(SEXTANTSolver):
    def __init__(self, env_model, explorer_model, viz=None, optimize_on='Energy', heuristic_accelerate=1):
        cost_function = ExplorerCost(explorer_model, env_model, optimize_on, heuristic_accelerate)
        super(astarSolver, self).__init__(env_model, cost_function, viz)

    def solve(self, startpoint, endpoint):
        env_model = self.env_model
        if env_model.elt_hasdata(startpoint) and env_model.elt_hasdata(endpoint):
            search = sextantSearch(startpoint, endpoint)
            node1, node2 = MeshSearchElement(env_model.getMeshElement(startpoint)), \
                           MeshSearchElement(env_model.getMeshElement(endpoint))
            solution_path, expanded_items = aStarSearch(node1, node2, self.cost_function, self.viz)
            raw, nodes = solution_path
            if len(raw) == 0:
                coordinates = []
            else:
                coordinates = GeoPolygon(env_model.ROW_COL, *np.array(raw).transpose())\
                    .to(LONG_LAT).transpose().tolist()
            search.addresult(raw, nodes, coordinates, expanded_items)
            self.searches.append(search)
            return search
        else:
            return False

if __name__ == '__main__':
    from pextant.settings import *
    from pextant.EnvironmentalModel import GDALMesh
    from pextant.ExplorerModel import Astronaut
    from pextant.mesh.MeshVisualizer import ExpandViz, MeshVizM
    jloader = WP_HI[7]
    waypoints = jloader.get_waypoints()
    envelope = waypoints.geoEnvelope()#.addMargin(0.5, 30)
    env_model = HI_DEM_LOWQUAL.loadSubSection(envelope, maxSlope=35)
    astronaut = Astronaut(80)
    cost_function = ExplorerCost(astronaut, env_model, "Energy", 1)
    solver = astarSolver(env_model, cost_function, ExpandViz(env_model, 10000))

    waypointseasy = [GeoPoint(env_model.ROW_COL, 1,1), GeoPoint(env_model.ROW_COL, 5,10),
                     GeoPoint(env_model.ROW_COL, 5,15)]

    segmentsout, rawpoints, items = solver.solvemultipoint(waypoints)
    jsonout = jloader.add_search_sol(segmentsout, True)

    matviz = MeshVizM()
    solgrid = np.zeros((env_model.y_size, env_model.x_size))
    for i in rawpoints:
        solgrid[i] = 1
    matviz.viz(solgrid)
