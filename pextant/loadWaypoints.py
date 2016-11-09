import json
import pandas as pd
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