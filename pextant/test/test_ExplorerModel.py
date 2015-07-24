import unittest
from pextant.api import *

class TestAstronaut(unittest.TestCase):
	
	def setUp(self):
		self.Explorer = Astronaut(70, gravity = 9.81)
		
	def test_type(self):
		self.assertEqual(self.Explorer.type, 'Astronaut')
		
	def test_costFunctions(self):
		pass
		
class TestRover(unittest.TestCase):
	
	def setUp(self):
		self.Explorer = Rover(100, gravity = 9.81, constant_speed = 15, additional_energy = 1500)
		
	def test_type(self):
		self.assertEqual(self.Explorer.type, 'Rover')

	def test_costFunctions(self):
		pass
		
class TestBASALTExplorer(unittest.TestCase):
	
	def setUp(self):
		pass
		
	def test_type(self):
		pass
		
	def test_costFunctions(self):
		pass
		
if __name__ == "__main__":
	suite1 = unittest.TestLoader().loadTestsFromTestCase(TestAstronaut)
	suite2 = unittest.TestLoader().loadTestsFromTestCase(TestRover)
	suite3 = unittest.TestLoader().loadTestsFromTestCase(TestBASALTExplorer)

	suite = unittest.TestSuite([suite1, suite2, suite3])
	
	unittest.TextTestRunner(verbosity=2).run(suite)