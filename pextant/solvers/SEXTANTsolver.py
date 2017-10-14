from pextant.lib.geoshapely import LONG_LAT
import numpy as np

class SEXTANTSolver(object):
    def __init__(self, environmental_model, cost_function, viz):
        self.env_model = environmental_model
        self.cost_function = cost_function
        self.viz = viz
        self.searches = []

    def solve(self, start_point, end_point):
        pass

    def solvemultipoint(self, waypoints):
        rawpoints = []
        itemssrchd = []
        search_results = []
        for i in range(len(waypoints) - 1):
            search_result = self.solve(waypoints[i], waypoints[i + 1])
            search_results.append(search_result)
            rawpoints += search_result.raw
            itemssrchd += search_result.expanded_items
        return search_results, np.array(rawpoints).transpose(), itemssrchd

class sextantSearchList(object):
    def __init__(self):
        self.list = []

    def append(self, elt):
        self.list.append(elt)

    def raw(self):
        pass
        #return np.array([search.raw for search in self.list])

class sextantSearch(object):
    def __init__(self, startpoint, endpoint):
        self.startpoint = startpoint
        self.endpoint = endpoint

    def addresult(self, raw, nodes, coordinates, expanded_items):
        self.namemap = {
            'time': ['timeList','totalTime'],
            'pathlength': ['distanceList','totalDistance'],
            'energy': ['energyList','totalEnergy']
        }
        #self.searches = []
        self.nodes = nodes
        self.raw = raw
        self.coordinates = coordinates
        self.expanded_items = expanded_items

    def tojson(self):
        out = {}
        coordinates = self.coordinates.to(LONG_LAT).transpose().tolist()
        out["geometry"] = {
            'type': 'LineString',
            'coordinates': coordinates
        }
        results = {}
        for k, v in self.namemap.items():
            results.update({v[0]:[],v[1]:0})
        for i, mesh_srch_elt in enumerate(self.nodes):
            derived = mesh_srch_elt.derived
            for k, v in derived.items():
                results[self.namemap[k][0]].append(v)
        for k, v in self.namemap.items():
            results[v[1]] = sum(results[v[0]])
        out["derivedInfo"] = results
        return out

    def tocsv(self, coordstype=LONG_LAT):
        sequence = []
        coords = self.coordinates.to(coordstype).transpose().tolist()
        for i, mesh_srch_elt in enumerate(self.nodes):
            if i != 0:
                row_entry = [i==1 or i==len(coords)-1] #True if it's the first or last entry
                row_entry += coords[i] + [mesh_srch_elt.mesh_element.z]
                derived = mesh_srch_elt.derived
                row_entry += [derived['pathlength'], derived['time'], derived['energy']]
                sequence += [row_entry]
        return sequence

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