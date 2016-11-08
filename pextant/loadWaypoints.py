import json
import pandas as pd

def loadPoints(filename):
    filename = 'waypoints/MD10_EVA10_Stn18_Stn23_X.json'
    with open(filename) as data_file:
        data = json.load(data_file)

    ways_and_segments = data['sequence']
    s = pd.DataFrame(ways_and_segments)
    waypoints = s[s['type']=='Station']['geometry']
    w = waypoints.values.tolist()
    latlongFull = pd.DataFrame(w)
    latlongInter = latlongFull['coordinates'].values.tolist()
    waypointslatlong = pd.DataFrame(latlongInter, columns=['longitude','latitude'])