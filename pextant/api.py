import csv
import json
import logging

from ExplorationObjective import ActivityPoint
from pextant.lib.geoshapely import *
from pextant.solvers.astarSEXTANT import fullSearch, ExplorerCost
from pextant.analysis.loadWaypoints import loadPointsOld
logger = logging.getLogger()

class Segment:
    def __init__(self):
        self.json = ""
        self.coordinates = []
        self.distancelist = []
        self.energylist = []
        self.timelist = []

    def to_JSON(self):
        return {
            "type": "Segment",
            "geometry": {
                "type": "LineString",
                "coordinates": self.coordinates
            },
            "derivedInfo": {
                "distanceList": self.distancelist,
                "energyList": self.energylist,
                "timeList": self.timelist,
                "totalDistance": sum(self.distancelist),
                "totalTime": sum(self.timelist),
                "totalEnergy": sum(self.energylist)
            }
        }


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
        self.map = environmental_model

    def aStarCompletePath(self, optimize_on, waypoints, returnType="JSON", dh=None, fileName=None ):
        pass

    def completeSearch(self, optimize_on, waypoints, returnType="JSON", dh=None, fileName=None ):
        """
        Returns a tuple representing the path and the total cost of the path.
        The path will be a list. All activity points will be duplicated in
        the returned path.

        waypoints is a list of activityPoint objects, in the correct order. fileName is
        used when we would like to write stuff to a file and is currently necessary
        for csv return types.
        """
        env_model = self.map
        explorer_model = self.explorer
        cost_function = ExplorerCost(explorer_model, env_model, optimize_on)
        path, fullcost, itemssrchd = fullSearch(waypoints, env_model, cost_function)
        fullpath = path[0]

        if returnType == "tuple":
            return (fullpath, fullcost, sum(fullcost))
        elif returnType == "JSON":
            data = self._toJSON(fullpath, optimize_on, waypoints)
            if fileName:
                with open(fileName, 'w') as outfile:
                    json.dump(data, outfile, indent=4)
            return data
        elif returnType == "csv":
            sequence = self._toCSV(fullpath, optimize_on, waypoints)
            print sequence
            if fileName:
                with open(fileName, 'wb') as csvfile:
                    writer = csv.writer(csvfile)
                    for row in sequence:
                        writer.writerow(row)
            return sequence


    def completeSearchFromJSON(self, optimize_on, jsonInput, returnType="JSON", fileName=None, algorithm="A*",
                               numTestPoints=0):
        waypoints, parsed_json = loadPointsOld(jsonInput)

        if algorithm == "A*":
            path = self.completeSearch(optimize_on, waypoints, returnType, fileName)

        for i, element in enumerate(parsed_json):
            if element["type"] == "Segment":
                parsed_json[i]["derivedInfo"] = path[i]["derivedInfo"]
                parsed_json[i]["geometry"] = path[i]["geometry"]

        return json.dumps(parsed_json)

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
            ap = waypoints.pop(0)
            element = {}

            element["geometry"] = {"coordinates": ap.to(LONG_LAT, list), "type": "Point"}
            element["type"] = "Station"
            element["UUID"] = 0

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
            row, col = point
            geopoint = GeoPoint(self.map.ROW_COL, row, col)
            lonlat = geopoint.to(LONG_LAT, list)
            if i == 0:  # first element is a station
                sequence.append(nextStation(lineString, True))
                lineString = empty()
                lineString["geometry"]["coordinates"].append(lonlat)
            elif point == path[i - 1]:  # point is identical to the previous one and is thus a station
                sequence.append(nextStation(lineString))
                lineString = empty()
                lineString["geometry"]["coordinates"].append(lonlat)
            else:  # This point is not a station and is thus a point inside the path
                lineString["geometry"]["coordinates"].append(lonlat)

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


if __name__ == '__main__':
    from pextant.analysis.loadWaypoints import loadPoints
    from ExplorerModel import Astronaut
    from EnvironmentalModel import GDALMesh

    hi_low = GDALMesh('maps/HI_lowqual_DEM.tif')
    waypoints = loadPoints('waypoints/HI_13Nov16_MD7_A.json')
    env_model = hi_low.loadMapSection(waypoints.geoEnvelope())
    astronaut = Astronaut(80)
    pathfinder = Pathfinder(astronaut, env_model)
    out = pathfinder.aStarCompletePath('Energy', waypoints)
    print out