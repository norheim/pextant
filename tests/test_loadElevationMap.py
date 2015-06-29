from sextant import *

import numpy as np
import math

import json

from osgeo import gdal

a = np.array([[1., 6, 5, 2, 5, 6, 3],
			  [1, 3, 6, 5, 1, 2, 5],
			  [1, 9, 9, 9 ,9, 1, 3],
			  [1, 9, 9, 9, 9, 1, 4],
			  [4, 9, 9, 9, 2, 1, 4],
			  [1, 2, 2, 1, 3, 2, 2],
			  [1, 2, 2, 1, 4, 5, 3]])

map = EnvironmentalModel(a, 20, 15, 'Earth')

print map.convertToLatLong((0, 0)).latitude
print map.convertToLatLong((0, 0)).longitude

print map.convertToLatLong((5, 5)).latitude
print map.convertToLatLong((5, 5)).longitude

ap1 = ActivityPoint((1, 1), 4.6, {"yolo": 3})
ap2 = ActivityPoint((4, 4), 5.2, {"askfd": "asdfohi"})
ap3 = ActivityPoint((6, 4), 3.3, {"gg": "nore"})

astronaut = Astronaut(120, 9.81, None)

P = Pathfinder(astronaut, map, [ap1, ap2, ap3])

#print np.round(P.map.slopes)
#path = P.completePath('Energy')
#print path
#path_energies = P.analyzePath(path[0], 'Energy')
#print path_energies

#print P.toJSON("Energy")