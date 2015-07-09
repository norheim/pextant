import numpy
from osgeo import gdal
import unittest
from pextant.api import *

inferno_map_address = 'datasets\INFERNO-DEM.tif'

ds = gdal.Open(inferno_map_address)
band = ds.GetRasterBand(1)
map = band.ReadAsArray(1000, 1000, 20, 10).astype(numpy.float)

GeoInfo = ds.GetGeoTransform()

class TestEnvironmentalModelMethods(unittest.TestCase):

	def setUp(self):
		self.EM = EnvironmentalModel(map, 0.0745119, 15, "Earth", LatLongCoord(GeoInfo[3], GeoInfo[0]))

	def test_initialize(self):
		self.assertEqual(self.EM.resolution, 0.0745119)
		self.assertEqual(self.EM.elevations.shape, (10, 20))
		self.assertFalse(self.EM.obstacles[0][3])
		self.assertTrue(self.EM.obstacles[2][3])
	
	def test_obstacleFunctions(self):
		self.EM.setMaxSlope(25)
		self.assertTrue(self.EM.obstacles[0][3])
		self.assertTrue(self.EM.obstacles[2][3])
		
		self.EM.setMaxSlope(10)
		self.assertFalse(self.EM.obstacles[0][3])
		self.assertFalse(self.EM.obstacles[2][3])
		
		self.EM.setMaxSlope(15)
		self.assertFalse(self.EM.obstacles[0][3])
		self.assertTrue(self.EM.obstacles[2][3])
		
		self.EM.setObstacle((2, 3))
		self.assertFalse(self.EM.obstacles[2][3])
		
		self.EM.setMaxSlope(20)
		self.assertFalse(self.EM.obstacles[2][3])
		
		self.EM.setMaxSlope(15)
		self.assertFalse(self.EM.obstacles[2][3])
		
		self.EM.eraseObstacle((2, 3))
		self.assertTrue(self.EM.obstacles[2][3])
	
	def test_getFunctions(self):
		elev = self.EM.elevations[5][10]
		slope = self.EM.slopes[5][10]
		self.assertEqual(self.EM.getElevation((5, 10)), elev)
		self.assertEqual(self.EM.getSlope((5, 10)), slope)
	
	def test_conversion(self): #Unfortunately the precision of utm is not that high...only on the decimeter scale...

		UTMCo = UTMCoord(332107.99, 4692261.58, 19, 'T')
		LL1 = self.EM.convertToLatLong(UTMCo)
		UTMCo2 = self.EM.convertToUTM(LL1)
		
		print UTMCo
		print LL1
		print UTMCo2
		
		self.assertEqual(UTMCo.zone, UTMCo2.zone)
		self.assertEqual(UTMCo.zoneLetter, UTMCo2.zoneLetter)
		self.assertAlmostEqual(UTMCo.easting, UTMCo2.easting, 3)
		self.assertAlmostEqual(UTMCo.northing, UTMCo2.northing, 3) # This guarantees millimeter precision

class TestLatLong(unittest.TestCase):
	pass


class TestUTMCoord(unittest.TestCase):
	pass


if __name__ == "__main__":
	suite = unittest.TestLoader().loadTestsFromTestCase(TestEnvironmentalModelMethods)
	unittest.TextTestRunner(verbosity=2).run(suite)