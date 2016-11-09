from api import *
from EnvironmentalModel import *
from ExplorerModel import Astronaut
from bokeh.plotting import figure, output_file, show
from bokeh.io import hplot
from loadWaypoints import loadPoints

output_file("lines.html", title="line plot example")

dem_path = 'maps/HI_lowqual_DEM.tif'
dataset, info = loadElevationsLite(dem_path)
#print(info)

import json
import pandas as pd
pd.options.display.max_rows = 5

waypoints = loadPoints('waypoints/HI_sextant_testing2_B.json')

XY = Cartesian(info["nw_geo_point"], info["resolution"])
nw_corner = waypoints.geoEnvelope().addMargin(XY,10).upper_left
se_corner = waypoints.geoEnvelope().addMargin(XY,10).lower_right

EM2 = loadElevationMap(dem_path, maxSlope=15, planet='Earth', nw_corner=nw_corner, se_corner=se_corner,
                          desired_res=info["resolution"], no_val=-10000)

astronaut = Astronaut(45)
ap = []
for waypoint in waypoints.to(LAT_LONG).transpose():
    lat, long = waypoint
    ap.append(ActivityPoint(GeoPoint(LAT_LONG,lat,long), 0))

P = Pathfinder(astronaut, EM2)
out = P.aStarCompletePath([0, 0, 1], ap, 'tuple')
print out