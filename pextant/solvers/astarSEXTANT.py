import math
from astar import aStarSearchNode, aStarCostFunction
import numpy as np
from collections import defaultdict
from pextant.lib.geoshapely import GeoPoint, GeoPolygon, LONG_LAT
from shapely.geometry import mapping

class MeshSearchElement(aStarSearchNode):
    def __init__(self, mesh_element, parent=None):
        self.mesh_element = mesh_element
        state = (mesh_element.row, mesh_element.col)
        self.derived = {} #the point of this is to store in memory expensive calculations we might need later
        super(MeshSearchElement, self).__init__(state, parent)

    def getChildren(self):
        return [MeshSearchElement(mesh_element, self)
                for mesh_element in self.mesh_element.getNeighbours().collection]

    def __str__(self):
        return str(self.mesh_element)

class ExplorerCost(aStarCostFunction):
    def __init__(self, astronaut, environment, optimize_on):
        super(ExplorerCost, self).__init__()
        self.explorer = astronaut
        self.map = environment
        self.optimize_vector = astronaut.optimizevector(optimize_on)

    def getHeuristicCost(self, node):
        start_row, start_col = node.state
        end_row, end_col = self.end_node.state
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
        heuristic_cost = 10*(d * math.sqrt(2) * h_diagonal + d * (h_straight - 2 * h_diagonal))
        # This is just Euclidean distance
        #heuristic_cost = d * math.sqrt((start_row - end_row) ** 2 + (start_col - end_col) ** 2)

        return heuristic_cost

    def getCostBetween(self, fromnode, tonode):
        """
            Given the start and end states, returns the cost of travelling between them.
            Allows for states which are not adjacent to each other.

            optimize_vector is a list or tuple of length 3, representing the weights of
            Distance, Time, and Energy
        """
        optimize_weights = self.optimize_vector
        explorer  = self.explorer
        from_elt, to_elt = fromnode.mesh_element, tonode.mesh_element
        slope, path_length = from_elt.slopeTo(to_elt)
        time = explorer.time(path_length, slope)

        #TODO: rewrite this so not all functions need to get evaluated(expensive)
        optimize_vector = np.array([
            path_length,
            time,
            explorer.energyCost(path_length, slope, self.map.getGravity())
        ])
        cost = np.dot(optimize_vector, optimize_weights)
        tonode.derived = {
            'time': time,
            'energy': cost,
            'pathlength': path_length
        }
        return cost

class sextantSearch:
    def __init__(self, env_model):
        self.namemap = {
            'time': ['timeList','totalTime'],
            'pathlength': ['distanceList','totalDistance'],
            'energy': ['energyList','totalEnergy']
        }
        self.searches = []
        self.env_model = env_model

    def tojson(self, geopolygon, nodes):
        out = {}
        out["geometry"] = {
            'type': 'LineString',
            'coordinates': geopolygon.to(LONG_LAT).transpose().tolist()
        }
        results = {}
        for i, mesh_srch_elt in enumerate(nodes):
            derived = mesh_srch_elt.derived
            for k, v in derived.items():
                results.setdefault(self.namemap[k][0],[]).append(v)
        for k, v in self.namemap.items():
            results[v[1]] = sum(results[v[0]])
        out["derivedInfo"] = results
        return out

    def search(self, geopoint1, geopoint2, cost_function, viz):
        node1, node2  = MeshSearchElement(self.env_model.getMeshElement(geopoint1)), \
                        MeshSearchElement(self.env_model.getMeshElement(geopoint2))
        solution_path, expanded_items = aStarSearch(node1, node2, cost_function, viz)
        raw, nodes = solution_path
        geopolygon = GeoPolygon(self.env_model.ROW_COL, *np.array(raw).transpose())
        self.searches.append({
            'raw': raw,
            'geopolygon': geopolygon,
            'nodes': nodes,
            'expanded_items':expanded_items,
        })
        return raw, geopolygon, nodes, expanded_items


def fullSearch(waypoints, env_model, cost_function, viz=None):
    segments = []
    rawpoints = []
    itemssrchd = []
    solver = sextantSearch(env_model)
    for i in range(len(waypoints)-1):
        raw, geopolygon, nodes, expanded_items = solver.search(waypoints[i], waypoints[i+1], cost_function, viz)
        segments.append(solver.tojson(geopolygon, nodes))
        rawpoints += raw
        itemssrchd += expanded_items
    return segments, rawpoints, itemssrchd

import matplotlib.pyplot as plt
class ExpandViz(object):
    def __init__(self, rows, cols):
        self.expandedgrid = np.zeros((rows, cols))
        self.counter = 0

    def add(self, state, cost):
        self.expandedgrid[state] = cost
        self.counter += 1

        if self.counter % 100 == 0:
            print self.counter

        if self.counter % 10000 == 0 and False:
            plt.matshow(viz.expandedgrid)
            plt.show()


if __name__ == '__main__':
    from pextant.analysis.loadWaypoints import JSONloader
    from pextant.EnvironmentalModel import GDALMesh
    from pextant.ExplorerModel import Astronaut
    from astar import aStarSearch
    import numpy.ma as ma
    hi_low = GDALMesh('../../data/maps/Hawaii/HI_air_imagery.tif')
    jloader = JSONloader('../../data/waypoints/HI_13Nov16_MD7_A.json')
    waypoints = jloader.get_waypoints()
    env_model = hi_low.loadMapSection(waypoints.geoEnvelope())
    import matplotlib.pyplot as plt
    elevations = env_model.dataset
    elv = ma.masked_array(elevations, elevations < 0)
    plt.matshow(elv)
    plt.show()
    astronaut = Astronaut(80)
    cost_function = ExplorerCost(astronaut, env_model, "Energy")
    # env_model.generateRelief(50)
    waypointseasy = [GeoPoint(env_model.ROW_COL, 1,1), GeoPoint(env_model.ROW_COL, 5,10),
                     GeoPoint(env_model.ROW_COL, 5,15)]
    waypointsproblem = [waypoints[2], waypoints[3]]
    viz = ExpandViz(env_model.numRows, env_model.numCols)
    segmentsout, rawpoints, items = fullSearch(waypoints, env_model, cost_function, viz)
    jsonout = jloader.add_search_sol(segmentsout, True)
    #out = [(51, 283), (50, 283), (50, 282), (49, 281), (48, 280), (47, 280), (47, 279), (46, 279), (45, 279),
    # (44, 278), (44, 277), (44, 276), (43, 275), (42, 274), (41, 273), (40, 272), (40, 271), (40, 270), (40, 269), (40, 268), (40, 267), (40, 266), (39, 265), (38, 264), (37, 263), (36, 262), (35, 261), (35, 260), (34, 259), (34, 258), (34, 257), (34, 256), (33, 255), (32, 255), (31, 255), (30, 255), (29, 255), (28, 255), (27, 255), (26, 255), (26, 254), (25, 253), (24, 253), (23, 253), (22, 253), (21, 252), (20, 252), (19, 252), (18, 252), (17, 251), (16, 251), (15, 251), (14, 252), (13, 252), (12, 252), (11, 252), (10, 252), (9, 251), (8, 251), (8, 250), (8, 249), (7, 249), (6, 250), (6, 249), (6, 248), (5, 249), (4, 250), (4, 249), (3, 249), (3, 248), (3, 247), (2, 246), (1, 247), (0, 248), (0, 247), (0, 246)]
    solgrid = np.zeros((env_model.numRows, env_model.numCols))
    for i in rawpoints:
        solgrid[i] = 1
    plt.matshow(solgrid)
    plt.show()
    print(jsonout)
