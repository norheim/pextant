import json

import pandas as pd

from pextant.lib.geoshapely import *


def loadPointsOld(filename):
    parsed_json = json.loads(jsonInput)
    waypoints = []

    for element in parsed_json:  # identify all of the waypoints
        if element["type"] == "Station":
            lon, lat = element["geometry"]["coordinates"]
            time_cost = element["userDuration"]
            waypoints.append(GeoPolygon(LAT_LONG, lon, lat))

    return waypoints, parsed_json

def loadPoints(filename):
    with open(filename) as data_file:
        data = json.load(data_file)

    ways_and_segments = data['sequence']
    s = pd.DataFrame(ways_and_segments)
    waypoints = s[s['type']=='Station']['geometry']
    w = waypoints.values.tolist()
    latlongFull = pd.DataFrame(w)
    latlongInter = latlongFull['coordinates'].values.tolist()
    waypointslatlong = pd.DataFrame(latlongInter, columns=['longitude','latitude'])
    return GeoPolygon(LAT_LONG, waypointslatlong['latitude'].values, waypointslatlong['longitude'].values)

def loadSegments(filename):
    with open(filename) as data_file:
        data = json.load(data_file)

    ways_and_segments = data['sequence']
    s = pd.DataFrame(ways_and_segments)
    waypoints = s[s['type']=='Segment']['geometry']
    w = waypoints.values.tolist()
    latlongFull = pd.DataFrame(w)
    latlongInter = latlongFull['coordinates'].values.tolist()
    waypointslatlong = np.array(latlongInter[0])
    return GeoPolygon(LAT_LONG, waypointslatlong[:,1], waypointslatlong[:,0])
