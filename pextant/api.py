from ExplorationObjective import ActivityPoint
from geoshapely import *
import logging
import csv
import heapq
import numpy as np
import math
import json
from scipy.optimize import fmin_slsqp

from osgeo import gdal, osr

logger = logging.getLogger()


class Pathfinder:
    """
    This class performs the A* path finding algorithm and contains the Cost Functions. Also includes
    capabilities for analysis of a path.

    This class still needs performance testing for maps of larger sizes. I don't believe that
    we will be doing anything extremely computationally intensive though.

    Current cost functions are Time, Distance, and (Metabolic) Energy. It would be useful to be able to
    optimize on other resources like battery power or water sublimated, but those are significantly more
    difficult because they depend on shadowing and was not implemented by Aaron.
    """

    def __init__(self, explorer_model, environmental_model):
        self.explorer = explorer_model
        self.map = environmental_model  # a 2d array

    def _goalTest(self, node, endnode):
        """
        Determines the criteria for "reaching" a node. Could potentially be modified to something like
        return distance < 5
        """
        return node.state == endnode.state  # this should be the same no matter what value we are optimizing on

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

    def _aStarCostFunction(self, start_state, end_state, optimize_on):
        """
        Given the start and end states, returns the cost of travelling between them.
        Allows for states which are not adjacent to each other.

        Note that this uses start and end coordinates rather than nodes.

        optimize_vector is a list or tuple of length 3, representing the weights of
        Distance, Time, and Energy
        """
        optimize_vector = self._vectorize(optimize_on)

        r = self.map.resolution
        start_row, start_col = start_state
        end_row, end_col = end_state
        start_elev, end_elev = self.map.getElevation(start_state), self.map.getElevation(end_state)

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

    def _heuristic(self, current_node, end_node, optimize_on, search_algorithm):
        """
        A basic heuristic to speed up the search
        """
        # For A* search
        start_row, start_col = current_node.state
        end_row, end_col = end_node.state

        if search_algorithm == "Field D*":
            start_row, start_col = current_node.coordinates
            end_row, end_col = end_node.coordinates

        optimize_vector = self._vectorize(optimize_on)

        h_diagonal = min(abs(start_row - end_row),
                         abs(start_col - end_col))  # max number of diagonal steps that can be taken
        h_straight = abs(start_row - end_row) + abs(start_col - end_col)  # Manhattan distance

        # D represents the cost between two consecutive nodes
        d = 0

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

        if search_algorithm == "A*":
            # Patel 2010. See page 49 of Aaron's thesis
            heuristic_cost = d * math.sqrt(2) * h_diagonal + d * (h_straight - 2 * h_diagonal)
        elif search_algorithm == "Field D*":
            # This is just Euclidean distance
            heuristic_cost = d * math.sqrt((start_row - end_row) ** 2 + (start_col - end_col) ** 2)

        return heuristic_cost

    def _aStarGetNeighbors(self, search_node, optimize_on):
        # returns a list of states that can be travelled to
        # basically just the 8 surrounding tiles, if they are inbounds and not obstacles
        row = search_node.state[0]
        col = search_node.state[1]
        children = [(row + 1, col), (row + 1, col + 1), (row + 1, col - 1), (row, col + 1), (row, col - 1),
                    (row - 1, col + 1), (row - 1, col), (row - 1, col - 1)]
        valid_children = [child for child in children if self.map.isPassable(child)]
        return [aStarSearchNode(state, search_node, search_node.cost +
                                self._aStarCostFunction(search_node.state, state, optimize_on)) for state in
                valid_children]

    def aStarSearch(self, start_node, end_node, optimize_on, p=None, dh=None):
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
        # raise errors if start/end nodes are out of bounds
        if not self.map._inBounds(start_node.state):
            raise IndexError("The location ", start_node.state, "is out of bounds")
        elif not self.map._inBounds(end_node.state):
            raise IndexError("The location ", end_node.state, "is out of bounds")

        if self._goalTest(start_node, end_node):
            return start_node.getPath()
        agenda = []
        node_cost = start_node.cost
        heuristic = self._heuristic(start_node, end_node, optimize_on, "A*")
        estimated_cost = node_cost + heuristic
        heapq.heappush(agenda, (estimated_cost, start_node))
        # agenda contains pairs (cost, node)
        expanded = set()
        while len(agenda) > 0:
            priority, node = heapq.heappop(agenda)
            if node.state not in expanded:
                expanded.add(node.state)
                if p is not None:
                    p[0].circle(node.state[1], dh-node.state[0], fill_color="green", line_color="green")
                    p[1].circle(node.state[1], dh - node.state[0], fill_color="green", line_color="green")
                    #mytext = Label(x=node.state[1], y=dh-node.state[0], text='here your text')
                    #p.add_layout(mytext)
                if self._goalTest(node, end_node):
                    return (node.getPath(), len(expanded), node.cost)
                for child in self._aStarGetNeighbors(node, optimize_on):
                    if child.state not in expanded and child.cost != float('inf'):
                        heapq.heappush(agenda,
                                       ((child.cost + self._heuristic(child, end_node, optimize_on, "A*"), child)))
        return (None, len(expanded), node.cost)

    def aStarCompletePath(self, optimize_on, waypoints, returnType="JSON", plot=None, dh=None, fileName=None ):
        """
        Returns a tuple representing the path and the total cost of the path.
        The path will be a list. All activity points will be duplicated in
        the returned path.

        waypoints is a list of activityPoint objects, in the correct order. fileName is
        used when we would like to write stuff to a file and is currently necessary
        for csv return types.
        """
        optimize_vector = self._vectorize(optimize_on)

        finalPath = []
        costs = []
        for i in range(len(waypoints)-1):
            segmentCost = 0

            node1 = aStarSearchNode(waypoints[i].coordinates.to(self.map.ROW_COL), None, 0)
            node2 = aStarSearchNode(waypoints[i+1].coordinates.to(self.map.ROW_COL), None, 0)
            partialPath = self.aStarSearch(node1, node2, optimize_vector, plot, dh)

            path, expanded, cost = partialPath

            if path:
                finalPath += path  # for now I'm not going to bother with deleting the duplicates
                # that will occur at every activity point. I think it might end up being useful
            else:
                return []
                #raise RuntimeError("Path not found between waypoints", waypoints[i].coordinates, "and",
                #                   waypoints[i + 1].coordinates)
            segmentCost += cost
            segmentCost += optimize_vector[1] * waypoints[i].duration

            costs.append(segmentCost)
        if returnType == "tuple":
            return (finalPath, costs, sum(costs))
        elif returnType == "JSON":
            data = self._toJSON(finalPath, optimize_on, waypoints)
            if fileName:
                with open(fileName, 'w') as outfile:
                    json.dump(data, outfile, indent=4)
            return data
        elif returnType == "csv":
            sequence = self._toCSV(finalPath, optimize_on, waypoints)
            print sequence
            if fileName:
                with open(fileName, 'wb') as csvfile:
                    writer = csv.writer(csvfile)
                    for row in sequence:
                        writer.writerow(row)
            return sequence

    def completeSearchFromJSON(self, optimize_on, jsonInput, returnType="JSON", fileName=None, algorithm="A*",
                               numTestPoints=0):
        parsed_json = json.loads(jsonInput)
        new_json = json.loads(jsonInput)
        waypoints = []

        for element in parsed_json:  # identify all of the waypoints
            if element["type"] == "Station":
                lon, lat = element["geometry"]["coordinates"]
                time_cost = element["userDuration"]
                waypoints.append(ActivityPoint(GeoPoint(LAT_LONG,lat, lon), time_cost, None))  # set the cost to time cost
        if algorithm == "A*":
            path = self.aStarCompletePath(optimize_on, waypoints, returnType, fileName)
        elif algorithm == "Field D*":
            path = self.fieldDStarCompletePath(optimize_on, waypoints, returnType, fileName, numTestPoints)

        for i, element in enumerate(new_json):
            if element["type"] == "Segment":
                new_json[i]["derivedInfo"] = path[i]["derivedInfo"]
                new_json[i]["geometry"] = path[i]["geometry"]

        return json.dumps(new_json)

    def _toJSON(self, path, optimize_on, waypoints):
        '''
        This returns the list following "sequence." It doesn't really make sense
        for SEXTANT to actually do parts of the JSON file such as "creator."
        '''
        sequence = []

        def empty():
            return {
                "geometry": {
                    "type": "LineString",
                    "coordinates": []
                },
                "derivedInfo": {
                    "distanceList": [],
                    "energyList": [],
                    "timeList": [],
                    "totalDistance": 0,
                    "totalTime": 0,

                    "totalEnergy": 0
                },
                "type": "Segment"
            }

        lineString = {}

        def nextStation(lineString, firstStation=False):  # firstStation denotes if the station is the first of the path
            AP = waypoints.pop(0)
            element = {}

            latLong = self.map.convertToLatLong(AP.coordinates)

            element["geometry"] = {"coordinates": latLong.longLatList(), "type": "Point"}
            element["type"] = "Station"
            element["UUID"] = AP.UUID

            # When we hit a station, we need to add the previous lineString to the sequence,
            # compute the total Dist, Time, and Energy
            if not firstStation:
                distSum = sum(lineString["derivedInfo"]["distanceList"])
                timeSum = sum(lineString["derivedInfo"]["timeList"])
                energySum = sum(lineString["derivedInfo"]["energyList"])

                lineString["derivedInfo"]["totalDistance"] = (distSum if distSum != float('inf') else "Infinity")
                lineString["derivedInfo"]["totalTime"] = (timeSum if timeSum != float('inf') else "Infinity")
                lineString["derivedInfo"]["totalEnergy"] = (energySum if energySum != float('inf') else "Infinity")

                for i in range(len(lineString["derivedInfo"]["distanceList"])):
                    if lineString["derivedInfo"]["distanceList"][i] == float('inf'):
                        lineString["derivedInfo"]["distanceList"][i] = "Infinity"
                    if lineString["derivedInfo"]["timeList"][i] == float('inf'):
                        lineString["derivedInfo"]["timeList"][i] = "Infinity"
                    if lineString["derivedInfo"]["energyList"][i] == float('inf'):
                        lineString["derivedInfo"]["energyList"][i] = "Infinity"

                sequence.append(lineString)

            return element

        for i, point in enumerate(path):
            # first set of if/elif statements creates the "Station" or "pathPoint" things,
            # second set of if/elif statements creates the "segments" connecting them
            if i == 0:  # first element is a station
                sequence.append(nextStation(lineString, True))
                lineString = empty()
                lineString["geometry"]["coordinates"].append(self.map.convertToLatLong(point).longLatList())
            elif point == path[i - 1]:  # point is identical to the previous one and is thus a station
                sequence.append(nextStation(lineString))
                lineString = empty()
                lineString["geometry"]["coordinates"].append(self.map.convertToLatLong(point).longLatList())
            else:  # This point is not a station and is thus a point inside the path
                latLongCoords = self.map.convertToLatLong(point).longLatList()

                lineString["geometry"]["coordinates"].append(latLongCoords)

                startState, endState = path[i - 1], path[i]
                distance = self._aStarCostFunction(startState, endState, [1, 0, 0])  # distance
                energy = self._aStarCostFunction(startState, endState, [0, 0, 1])  # energy
                time = self._aStarCostFunction(startState, endState, [0, 1, 0])  # time

                lineString["derivedInfo"]["distanceList"].append(distance)
                lineString["derivedInfo"]["energyList"].append(energy)
                lineString["derivedInfo"]["timeList"].append(time)

        # Call nextStation at the end to make the final point a station
        sequence.append(nextStation(lineString))

        return sequence

    def _toCSV(self, path, optimize_on, waypoints):

        sequence = []

        def nextStation():
            AP = waypoints.pop(0)

            latLong = self.map.convertToLatLong(AP.coordinates)
            elev = self.map.getElevation(AP.coordinates)

            return [True, latLong.longitude, latLong.latitude, elev]

        for i, point in enumerate(path):
            row = []
            # first set of if/elif statements creates the "Station" or "pathPoint" things,
            # second set of if/elif statements creates the "segments" connecting them
            if i == 0 or i == len(path) - 1:  # first and last elements are always station
                row += nextStation()
            elif point == path[i + 1]:  # point is identical to the next one and is thus a station
                pass
            elif point == path[i - 1]:
                row += nextStation()
            else:  # This point is not a station and is thus a "pathPoint," basically just a point inside the path
                latLongCoords = [self.map.convertToLatLong(point).longitude, self.map.convertToLatLong(point).latitude]
                elev = self.map.getElevation(point)
                row += ([False] + latLongCoords + [elev])

            if i == (len(path) - 1):  # We have reached the end of the path
                pass
            elif path[i] == path[i + 1]:  # This indicates the first coordinate in a station
                pass  # We add nothing to the csv
            else:  # Otherwise we need to add a segment
                startState, endState = path[i], path[i + 1]
                distance = self._aStarCostFunction(startState, endState, [1, 0, 0])
                energy = self._aStarCostFunction(startState, endState, [0, 0, 1])
                time = self._aStarCostFunction(startState, endState, [1, 0, 1])

                row += [distance, energy, time]

            if (i == (len(path) - 1)) or (path[i] != path[i + 1]):
                sequence += [row]

        sequence = [['isStation', 'x', 'y', 'z', 'distanceMeters', 'energyJoules', 'timeSeconds']] + sequence

        return sequence


class aStarSearchNode:
    '''
    This class is used to assist in the A* search
    '''

    def __init__(self, state, parent, cost=0):
        self.state = state
        self.parent = parent
        self.cost = cost

    def getPath(self):
        path = [self.state]
        node = self
        while node.parent:
            path.append(node.parent.state)
            node = node.parent

        path.reverse()
        return path
