from geopoint import *
import json
import pandas as pd
pd.options.display.max_rows = 5

with open('waypoints/MD10_EVA10_Stn18_Stn23_X.json') as data_file:
    data = json.load(data_file)
ways_and_segments = data['sequence']
s = pd.DataFrame(ways_and_segments)
waypoints = s[s['type']=='Station']['geometry']
w = waypoints.values.tolist()
latlongFull = pd.DataFrame(w)
latlongInter = latlongFull['coordinates'].values.tolist()
waypointslatlong = pd.DataFrame(latlongInter, columns=['longitude','latitude'])

waypoints = GeoPoint(LAT_LONG, waypointslatlong)
print(waypoints.to(UTM))
border = waypoints.square()
print(border.to(UTM))
origin = waypoints.upper_left()
print(origin.to(UTM))
COORD = DEMType(origin, 0.2)
print(border.to(COORD))
origin_xy = waypoints.to(COORD)
print(origin_xy)