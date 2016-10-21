from api import *
from EnvironmentalModel import *
from bokeh.plotting import figure, output_file, show
from bokeh.io import hplot

output_file("lines.html", title="line plot example")

dem_path = 'maps/hawaiiDEM.tif'

NWCorner = LatLongCoord(19.4025, -155.2750)
SECorner = LatLongCoord(19.3987, -155.2694)
print(NWCorner)
print(SECorner)

dem_map = loadElevationMap(dem_path, nw_corner=NWCorner, se_corner=SECorner)
dh, dw = dem_map.elevations.shape
print dw,dh
s1 = figure(title="simple line example", x_axis_label='x', y_axis_label='y', x_range=[0, dh], y_range=[0, dh])
s2 = figure(title="simple line example", x_axis_label='x', y_axis_label='y', x_range=[0, dh], y_range=[0, dh])
s1.image(image=[dem_map.elevations[::-1,:]], dw=dw, dh=dh, palette="Spectral11")
s2.image(image=[dem_map.obstacles[::-1,:]], dw=dw, dh=dh)
p = hplot(s1, s2)
astronaut = Astronaut(70)
P = Pathfinder(astronaut, dem_map)

latlong1 = LatLongCoord(19.401161895944455, -155.27348924428225)
utm1 = latLongToUTM(latlong1)
ap1 = ActivityPoint(latlong1, 0)
row1, col1 = dem_map.convertToRowCol(utm1)
print(row1,col1)

latlong2 = LatLongCoord(19.400835538227028, -155.27217370457942)
utm2 = latLongToUTM(latlong2)
ap2 = ActivityPoint(latlong2, 0)
row2, col2 = dem_map.convertToRowCol(utm2)
print(row2,col2)

s1.circle([col1,col2], [dh-row1,dh-row2], fill_color="yellow", line_color="yellow")
s2.circle([col1,col2], [dh-row1,dh-row2], fill_color="yellow", line_color="yellow")
show(p)
#final = P.aStarCompletePath([0, 0, 1], [ap1, ap2], 'tuple', [s1, s2], dh)
