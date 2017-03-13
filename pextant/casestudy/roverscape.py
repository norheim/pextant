from pextant.api import Pathfinder
from pextant.EnvironmentalModel import GDALMesh
from pextant.ExplorerModel import Astronaut
from pextant.lib.geoshapely import GeoEnvelope, GeoPoint, LAT_LONG
import matplotlib.pyplot as plt
import numpy as np

dem_map = GDALMesh('../../data/maps/Ames/Ames.tif')
env_model = dem_map.loadMapSection()
plt.matshow(env_model.dataset)
plt.show()
astronaut = Astronaut(70)
P = Pathfinder(astronaut, env_model)

waypoint1 = GeoPoint(env_model.ROW_COL,2,2)
waypoint2 = GeoPoint(env_model.ROW_COL,50,60)
segmentsout, rawpoints,_ = P.completeSearch('Energy', [waypoint1, waypoint2])
print segmentsout
solgrid = np.zeros((env_model.numRows, env_model.numCols))
for i in rawpoints:
    solgrid[i] = 1
plt.matshow(solgrid)
plt.show()
# print final
# path_x = [point[0] for point in final[0]]
# path_y = [point[1] for point in final[0]]
#
#
# plt.plot([149,71], [179,42], 'ro')
# plt.imshow(arr, cmap = 'viridis')
# plt.plot(path_y, path_x)
# plt.show()
