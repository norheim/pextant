from pextant.api import *
import numpy as np

elevs = np.array([[1, 3],
				  [5, 7]])

# map for testing out the Cost Function

class TestFieldDStarCostFunction(unittest.testcase):
	
	def setup(self):
		self.map = EnvironmentalModel(elevs, 20, 15)
		self.explorer = Astronaut(70)
		self.pf = Pathfinder(self.map, self.explorer)
	
	def testCostFunction(self):
		self.assertEqual(self.pf._fieldDStarCostFunction