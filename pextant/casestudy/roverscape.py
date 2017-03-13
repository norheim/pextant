from pextant.api import *
from pextant.EnvironmentalModel import loadElevationMap

import matplotlib.pyplot as plt
from osgeo import gdal



dem_map = loadElevationMap('maps/hwmidres.tif', maxSlope = 15,
                       planet = 'Earth', nw_corner= LatLongCoord(43.4638741,-113.5740501),
                       se_corner= LatLongCoord(43.4616207432,-113.571586718))

astronaut = Astronaut(70)
P = Pathfinder(astronaut, dem_map)
#
# map.convertToRowCol(LatLongCoord(37.41950672542035, -122.06479100044812))
# map.convertToRowCol(LatLongCoord(37.420130880587934, -122.06522283609952))
latlong1 = LatLongCoord(43.46329632, -113.57219385)
utm1 = latLongToUTM(latlong1)
row, col = dem_map.convertToRowCol(utm1)

latlong2 = LatLongCoord(43.46331013, -113.57266771)
utm2 = latLongToUTM(latlong2)
row2, col2 = dem_map.convertToRowCol(utm2)

ap1 = ActivityPoint(latlong1, 0)
ap2 = ActivityPoint(latlong2, 0)

final = P.aStarCompletePath([0, 0, 1], [ap1, ap2], 'tuple')
# print final
# path_x = [point[0] for point in final[0]]
# path_y = [point[1] for point in final[0]]
#
# ds = gdal.Open('Ames.tif')
# band = ds.GetRasterBand(1)
# arr = band.ReadAsArray()
#
# plt.plot([149,71], [179,42], 'ro')
# plt.imshow(arr, cmap = 'viridis')
# plt.plot(path_y, path_x)
# plt.show()
