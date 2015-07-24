from pextant.api import *
import unittest

info1 = {"depth": 0, 
		 "geometry": {
			 "coordinates": [
				-121.73915392390856, 50.8696259451617
			 ], 
                "type": "Point"
		 }, 
		 "id": "PLR000_A_STN01"}


class TestActivityPointMethods(unittest.TestCase):
	
	def setUp(self):
		self.AP = ActivityPoint((4, 6), duration = 10, information = info1)
		
	def test_setCoordinates(self):
		self.AP.setCoordinates((3, 7))
		self.assertEqual(3, self.AP.coordinates[0])
		self.assertEqual(7, self.AP.coordinates[1])
		
	def test_setDuration(self):
		self.AP.setDuration(23)
		self.assertEqual(self.AP.duration, 23)
		
	def test_addInformation(self):
		self.AP.addInformation({"type": "Station"})
		self.assertDictContainsSubset({"type": "Station"}, self.AP.information)
		
if __name__ == "__main__":
	suite = unittest.TestLoader().loadTestsFromTestCase(TestActivityPointMethods)
	
	unittest.TextTestRunner(verbosity=2).run(suite)