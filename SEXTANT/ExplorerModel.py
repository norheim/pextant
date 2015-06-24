import math

class Explorer:
	'''
	This class is a model for an arbitrary explorer model. It is very easily
	extensible (in fact, this is what we do for the astronaut and rover)
	'''
	def __init__(self, mass, gravity, parameters = None): # we initialize with mass and gravity
		self.mass = mass
		self.gravity = gravity # for basically any kind of (reasonable) explorer,
							   # mass and gravity stay constant and it makes sense
							   # to save them here
		self.parameters = parameters # parameters object, used for shadowing
	
	def distance(self, path_length):
		return path_length
	
	def velocity(self, slope):
		pass # should return something that's purely
			 # a function of slope
	
	def time(self, path_length, slope):
		return self.distance(path_length)/self.velocity(slope)
		
	def energyRate(self, path_length, slope):
		pass # this depends on velocity, time
	
	def energyCost(self, path_length, slope):
		return self.energyRate(path_length, slope)*self.time(path_length, slope)


class Astronaut(Explorer): #Astronaut extends Explorer
	def __init__(self, mass, gravity = 9.81, parameters = None):
		Explorer.__init__(self, mass, gravity, parameters)
		self.type = 'Astronaut'
		
	def velocity(self, slope): #slope is in degrees
							   #this is from Marquez 2008
							   #NOT the same slope as the one found in the slopes map
		if slope > 20:
			print "CAREFUL - slope is larger than 20 deg; velocity set to 0"
			return 0
		elif slope < -20:
			print "CAREFUL - slope is less than -20 deg; velocity set to 0"
			return 0
		elif slope > 15:
			return 0.05
		elif slope > 6:
			return -0.039 * slope + 0.634
		elif slope > 0:
			return -0.2 * slope + 1.6
		elif slope > -10:
			return 0.06 * slope + 1.6
		else:
			return 0.095 * alope + 1.95
			
	def energyRate(self, path_length, slope):
		'''
		Metabolic Rate Equations for a Suited Astronaut
		From Santee, 2001
		Literature review may be helpful?
		'''
		m = self.mass
		g = self.gravity
		v = self.velocity(slope)
		w_level = (3.28 * m + 71.1) * (0.661 * v * math.cos(math.radians(slope)) + 0.115)
		if slope == 0:
			w_slope = 0
		elif slope > 0:
			w_slope = 3.5 * m * g * v * math.sin(math.radians(slope)) # math.sin and math.cos are meant for radian measures!
		else:
			w_slope = 2.4 * m * g * v * math.sin(math.radians(slope)) * (0.3 ** (abs(slope) / 7.65))
		return w_level + w_slope
		
class Rover(Explorer): #Rover also extends explorer
	def __init__(self, mass, gravity = 9.81, parameters = None, constant_speed = 15, additional_energy = 1500):
		Explorer.__init__(mass, gravity, parameters)
		self.speed = constant_speed
		self.P_e = additional_energy # The collection of all additional electronic components on the rover
									 # Modelled as a constant, estimated to be 1500 W
									 # In future iterations, perhaps we can change this to depend on the
									 # activities being performed during the exploration
		self.type = 'Rover'
	
	def velocity(self, slope = 0):
		return self.speed #we assume constant velocity for the rover
		
	def energyRate(self, path_length, slope):
		'''
		Equations from Carr, 2001
		Equations developed from historical data
		Are all normalized to lunar gravity
		'''
		m = self.mass
		g = self.gravity
		v = self.velocity(slope)
		P_e = self.P_e
		w_level = 0.216 * m * v
		if slope == 0:
			w_slope = 0
		elif slope > 0:
			w_slope = 0.02628 * m * slope * (g / 1.62) * v
		elif slope < 0:
			w_slope = -0.007884 * m * slope * (g / 1.62) * v
		return w_level + w_slope + P_e
		
class explorerParameters:
	'''
	So I'm not sure if this is the best way to do things, but
	this class is basically purely for containing all of the
	parameters of a certain explorer, which is used in shadowing
	
	This may eventually become incorporated into a different type of
	energy cost function, especially for the rover. Currently the optimization
	on energy only takes into account metabolic energy, and not energy
	gained or lost from the sun and shadowing.
	
	The explorer class should be designed such that this is optional
	and only necessary if we wish to perform shadowing
	
	The input p should be a dictionary of parameters
	'''
	def __init__(self, type, p = None):
		if type == 'Astronaut' and p == None: #Default parameters for astronaut and rover
			self.dConstraint = 0
			self.tConstraint = 0
			self.eConstraint = 0
			self.shadowScaling = 120
			self.emissivityMoon = 0.95
			self.emissivitySuit = 0.837
			self.absorptivitySuit = 0.18
			self.solarConstant = 1367
			self.TSpace = 3
			self.VFSuitMoon = 0.5
			self.VFSuitSpace = 0.5
			self.height = 1.83
			self.A_rad = 0.093
			self.emissivityRad = 0
			self.m_dot = 0.0302
			self.cp_water = 4186
			self.h_subWater = 2594000
			self.conductivitySuit = 1.19
			self.inletTemp = 288
			self.TSuit_in = 300
			self.mSuit = 50
			self.cp_Suit = 1000
			self.suitBatteryHeat = 50
		elif type == 'Rover' and p == None:
			self.efficiency_SA = 0.26
			self.A_SA = 30
			self.batterySpecificEnergy = 145
			self.mBattery = 269
			self.electronicsPower = 1400
		elif type == 'Astronaut': # There should be a dictionary of parameters given
			self.dConstraint = p['dConstraint']
			self.tConstraint = p['tConstraint']
			self.eConstraint = p['eConstraint']
			self.shadowScaling = p['shadowScaling']
			self.emissivityMoon = p['emissivityMoon']
			self.emissivitySuit = p['emissivitySuit']
			self.absorptivitySuit = p['absorptivitySuit']
			self.solarConstant = p['solarConstant']
			self.TSpace = p['TSpace']
			self.VFSuitMoon = p['VFSuitMoon']
			self.VFSuitSpace = p['VFSuitSpace']
			self.height = p['height']
			self.A_rad = p['A_rad']
			self.emissivityRad = p['emissivityRad']
			self.m_dot = p['m_dot']
			self.cp_water = p['cp_water']
			self.h_subWater = p['h_subwater']
			self.conductivitySuit = p['conductivitySuit']
			self.inletTemp = p['inletTemp']
			self.TSuit_in = p['TSuit_in']
			self.mSuit = p['mSuit']
			self.cp_Suit = p['cp_Suit']
			self.suitBatteryHeat = p['suitBatteryHeat']
		elif type == 'Rover':
			self.efficiency_SA = p['efficiency_SA']
			self.A_SA = p['A_SA']
			self.batterySpecificEnergy = p['batterySpecificEnergy']
			self.mBattery = p['mBattery']
			self.electronicsPower = p['electronicsPower']