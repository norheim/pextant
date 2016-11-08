from api import *
from EnvironmentalModel import *
from bokeh.plotting import figure, output_file, show
from bokeh.io import hplot

output_file("lines.html", title="line plot example")

dem_path = 'maps/HI_lowqual_DEM.tif'
dataset, info = loadElevationsLite(dem_path)
#print(info)

import json
import pandas as pd
pd.options.display.max_rows = 5

with open('waypoints/HI16_Stn01_02_03_sextant_B.json') as data_file:
    data = json.load(data_file)
ways_and_segments = data['sequence']
s = pd.DataFrame(ways_and_segments)
waypoints = s[s['type']=='Station']['geometry']
w = waypoints.values.tolist()
latlongFull = pd.DataFrame(w)
latlongInter = latlongFull['coordinates'].values.tolist()
waypointslatlong = pd.DataFrame(latlongInter, columns=['longitude','latitude'])
waypoints = GeoPolygon(LAT_LONG, waypointslatlong['latitude'].values, waypointslatlong['longitude'].values)

easting,northing = np.array(waypoints.bounds).reshape((2,2)).transpose()
nw_corner = GeoPoint(UTM(info["zone"]), easting.min(), northing.max())
se_corner = GeoPoint(UTM(info["zone"]), easting.max(), northing.min())
nw_lat,nw_lon = nw_corner.to(LAT_LONG)
se_lat,se_lon = se_corner.to(LAT_LONG)

NWCorner = LatLongCoord(se_lat, se_lon)
SECorner = LatLongCoord(se_lat, se_lon)

EM2 = loadElevationMapExp(dem_path, maxSlope=15, planet='Earth', nw_corner=nw_corner, se_corner=se_corner,
                          desired_res=info["resolution"], no_val=-10000)

astronaut = Astronaut(45)
ap = []
for waypoint in np.array(waypoints):
    ap.append(ActivityPoint(UTMCoord(waypoint[0], waypoint[1], 5, 'N'), 0))

P = Pathfinder(astronaut, EM2)
out = P.aStarCompletePath([0, 0, 1], ap, 'tuple')
print out