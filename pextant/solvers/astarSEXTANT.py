import math
import numpy as np
from astar import aStarSearchNode, aStarCostFunction, aStarSearch
from pextant.lib.geoshapely import GeoPoint, GeoPolygon, LONG_LAT

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
    def __init__(self, raw, nodes, geopolygon, expanded_items):
        self.namemap = {
            'time': ['timeList','totalTime'],
            'pathlength': ['distanceList','totalDistance'],
            'energy': ['energyList','totalEnergy']
        }
        #self.searches = []
        self.nodes = nodes
        self.raw = raw
        self.geopolygon = geopolygon
        self.expanded_items = expanded_items

    def tojson(self):
        out = {}
        out["geometry"] = {
            'type': 'LineString',
            'coordinates': self.geopolygon.to(LONG_LAT).transpose().tolist()
        }
        results = {}
        for i, mesh_srch_elt in enumerate(self.nodes):
            derived = mesh_srch_elt.derived
            for k, v in derived.items():
                results.setdefault(self.namemap[k][0],[]).append(v)
        for k, v in self.namemap.items():
            results[v[1]] = sum(results[v[0]])
        out["derivedInfo"] = results
        return out

    def tocsv(self):
        sequence = []
        coords = self.geopolygon.to(LONG_LAT).transpose().tolist()
        for i, mesh_srch_elt in enumerate(self.nodes):
            row_entry = [i==1 or i==len(coords)-1] #True if it's the first or last entry
            row_entry += coords + [mesh_srch_elt.mesh_element.getElevevation()]
            derived = mesh_srch_elt.derived
            row_entry += [derived['pathlength'], derived['time'], derived['energy']]
            sequence += [row_entry]
        return sequence

def search(env_model, geopoint1, geopoint2, cost_function, viz):
    if env_model.has_data(geopoint1) and env_model.has_data(geopoint2):
        node1, node2  = MeshSearchElement(env_model.getMeshElement(geopoint1)), \
                    MeshSearchElement(env_model.getMeshElement(geopoint2))
        solution_path, expanded_items = aStarSearch(node1, node2, cost_function, viz)
        raw, nodes = solution_path
        geopolygon = GeoPolygon(env_model.ROW_COL, *np.array(raw).transpose())
        return sextantSearch(raw, nodes, geopolygon, expanded_items)

    raise ValueError("Start or end point out of map bounds, or in unreachable terrain")
    #self.searches.append({
    #    'raw': raw,
    #    'geopolygon': geopolygon,
    #    'nodes': nodes,
    #    'expanded_items':expanded_items,
    #})


from pextant.ExplorerModel import Astronaut
from pextant.EnvironmentalModel import GDALMesh
def fullSearch(waypoints, env_model, cost_function, viz=None):
    segment_searches = []
    rawpoints = []
    itemssrchd = []
    for i in range(len(waypoints)-1):
        search_result = search(env_model, waypoints[i], waypoints[i+1], cost_function, viz)
        segment_searches.append(search_result)
        rawpoints += search_result.raw
        itemssrchd += search_result.expanded_items
    return segment_searches, rawpoints, itemssrchd

import matplotlib.pyplot as plt
class ExpandViz(object):
    def __init__(self, env_model):
        self.env_model = env_model
        self.expandedgrid = np.zeros((env_model.numRows, env_model.numCols))
        self.counter = 0

    def draw(self):
        plt.matshow(self.env_model.slopes)
        #print(waypoints.to(env_model.COL_ROW))
        #plt.scatter(*waypoints.to(env_model.COL_ROW), c='r')
        plt.show()

    def add(self, state, cost):
        self.expandedgrid[state] = cost
        self.counter += 1

        if self.counter % 100 == 0:
            print self.counter

        if self.counter % 1000 == 0:
            plt.matshow(self.expandedgrid)
            plt.show()


if __name__ == '__main__':
    from pextant.analysis.loadWaypoints import JSONloader
    from pextant.EnvironmentalModel import GDALMesh
    from pextant.ExplorerModel import Astronaut
    import matplotlib.pyplot as plt
    import numpy.ma as ma
    hi_low = GDALMesh('../../data/maps/HI_lowqual_DEM.tif')
    jloader = JSONloader.from_file('../../data/waypoints/HI_13Nov16_MD7_A.json')
    waypoints = jloader.get_waypoints()
    env_model = hi_low.loadMapSection(waypoints.geoEnvelope().addMargin(0.5,30),35)
    # env_model.generateRelief(50)
    astronaut = Astronaut(80)
    cost_function = ExplorerCost(astronaut, env_model, "Energy")
    waypointseasy = [GeoPoint(env_model.ROW_COL, 1,1), GeoPoint(env_model.ROW_COL, 5,10),
                     GeoPoint(env_model.ROW_COL, 5,15)]
    viz = ExpandViz(env_model)
    segmentsout, rawpoints, items = fullSearch(waypoints, env_model, cost_function, viz)
    jsonout = jloader.add_search_sol(segmentsout, True)
    solgrid = np.zeros((env_model.numRows, env_model.numCols))
    for i in rawpoints:
        solgrid[i] = 1
    plt.matshow(solgrid)
    plt.show()
    print(jsonout)
