from pextant.api import *
import unittest

testfile = ('datasets\INFERNO-DEM.tif')

class TestPathfinder(unittest.TestCase):

	def setUp(self):
		map = EnvironmentalModel.loadElevationMap(testfile)
		explorer = Astronaut(120)
		AP1 = ActivityPoint((1300, 1300), 0, {})
		AP2 = ActivityPoint((1800, 1600), 10, {})
		AP3 = ActivityPoint((1450, 2100), 10, {})
		self.PF = Pathfinder(explorer, map, [AP1, AP2, AP3])