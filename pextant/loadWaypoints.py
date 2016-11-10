import json
import pandas as pd
import numpy as np
from geoshapely import *

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
