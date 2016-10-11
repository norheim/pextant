from api import *
import matplotlib.pyplot as plt
from osgeo import gdal

map = loadElevationMap('hawaiiDEM.tif', maxSlope = 15,\
					planet = 'Earth', NWCorner = None, SECorner = None, desiredRes = 0.5)

# astronaut = Astronaut(70)
# P = Pathfinder(astronaut, map)
#
# map.convertToRowCol(LatLongCoord(37.41950672542035, -122.06479100044812))
# map.convertToRowCol(LatLongCoord(37.420130880587934, -122.06522283609952))
# ap1 = ActivityPoint((179, 149), 0)
# ap2 = ActivityPoint((42.0, 71.0), 0)
#
# final = P.aStarCompletePath([0, 0, 1], [ap1, ap2], 'tuple')
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
