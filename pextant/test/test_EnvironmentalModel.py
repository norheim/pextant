import unittest
from dedupe.api import *

inferno_map_address = 'datasets\INFERNO_DEM.tif'

map = EnvironmentalModel.loadElevationMap(inferno_map_address)