import math
import logging
import numpy as np
from pextant.solvers.optimization import objectivefx, objectivize
logger = logging.getLogger()

class Explorer(object):
    '''
	This class is a model for an arbitrary explorer model
	'''

    def __init__(self, mass, uuid=None, parameters=None):  # we initialize with mass only
        # There used to be a gravity parameter, but it makes more sense to put it into EnvironmentalModel
        self.type = "N/A" #type for each explorer
        self.mass = mass
        self.UUID = uuid  # A type of ID system for every explorer
        self.parameters = parameters  # parameters object, used for shadowing
        self.analytics = dict()
        self.objectivefx = [
            ['Distance',self.distance, 'min'],
            ['Time', self.time, 'min'],
            ['Energy', self.energyCost, 'min']
        ]

    def optimizevector(self, arg):
        if isinstance(arg, str):
            id = next((i for i, item in enumerate(self.objectivefx) if arg in item),None)
            if id == None:
                id = 2
                #if the wrong syntax is passed in
            vector = np.zeros(len(self.objectivefx))
            vector[id] = 1
        else:
            vector = np.array(arg)
        return vector

    def distance(self, path_length):
        return path_length

    def velocity(self, slope):
        pass  # should return something that's purely a function of slope

    def time(self, path_length, slope):
        v = self.velocity(slope)
        if v != 0:
            return path_length / v
        else:
            return float('inf')  # Infinite time if zero velocity

    def energyRate(self, path_length, slope, g):
        return 0  # this depends on velocity, time

    def energyCost(self, path_length, slope, g):
        return self.energyRate(path_length, slope, g) * self.time(path_length, slope)


class Astronaut(Explorer):  # Astronaut extends Explorer
    def __init__(self, mass, uuid=None, parameters=None):
        super(Astronaut, self).__init__(mass, uuid, parameters)
        self.type = 'Astronaut'

    def velocity(self, slope):  # slope is in degrees, Marquez 2008
        if np.logical_or((slope >25), (slope <-25)).any():
            logger.debug("WARNING, there are some slopes out of bounds")
        if not isinstance(slope, np.ndarray):
            slope = np.array([slope])
        v = np.piecewise(slope,
                     [slope <= -20, (slope > -20) & (slope <= -10), (slope > -10) & (slope <= 0),
                      (slope > 0) & (slope <= 6 ), (slope>6) & (slope<=15), slope > 15],
                     [0.05, lambda slope: 0.095 * slope + 1.95, lambda slope: 0.06 * slope + 1.6,
                      lambda slope: -0.02 * slope + 1.6, lambda slope: -0.039 * slope + 0.634, 0.05])
        if v.shape[0] == 1:
            return v[0]
        else:
            return v.tolist()

    def energyRate(self, path_length, slope, g):
        '''
		Metabolic Rate Equations for a Suited Astronaut
		From Santee, 2001
		Literature review may be helpful?
		'''
        m = self.mass
        v = self.velocity(slope)
        w_level = (3.28 * m + 71.1) * (0.661 * v * math.cos(math.radians(slope)) + 0.115)
        if slope == 0:
            w_slope = 0
        elif slope > 0:
            w_slope = 3.5 * m * g * v * math.sin(math.radians(slope))
        else:
            w_slope = 2.4 * m * g * v * math.sin(math.radians(slope)) * (0.3 ** (abs(slope) / 7.65))
        return w_level + w_slope

class Rover(Explorer):  # Rover also extends explorer
    def __init__(self, mass, uuid=None, parameters=None, constant_speed=15, additional_energy=1500):
        Explorer.__init__(self, mass, uuid, parameters)
        self.speed = constant_speed
        self.P_e = additional_energy  # The collection of all additional electronic components on the rover
        # Modelled as a constant, estimated to be 1500 W
        # In future iterations, perhaps we can change this to depend on the
        # activities being performed during the exploration
        self.type = 'Rover'

    def velocity(self, slope=0):
        return self.speed  # we assume constant velocity for the rover

    def energyRate(self, path_length, slope, g):
        '''
		Equations from Carr, 2001
		Equations developed from historical data
		Are all normalized to lunar gravity
		'''
        m = self.mass
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


class BASALTExplorer(Explorer):
    '''
	Represents an explorer in the BASALT fields. Needs some data for the velocity function.
	energyRate should be the same as the Astronaut.
	'''

    def __init__(self, mass, parameters=None):
        Explorer.__init__(self, mass, parameters)
        self.type = 'BASALTExplorer'

    def velocity(self, slope=0):
        # Hopefully we can get this data after the first deployment
        pass

    def energyRate(self, path_length, slope, g):
        '''
		Metabolic Rate Equations for a Suited Astronaut
		From Santee, 2001
		Literature review may be helpful?
		'''
        m = self.mass
        v = self.velocity(slope)
        w_level = (3.28 * m + 71.1) * (0.661 * v * math.cos(math.radians(slope)) + 0.115)
        if slope == 0:
            w_slope = 0
        elif slope > 0:
            w_slope = 3.5 * m * g * v * math.sin(
                math.radians(slope))  # math.sin and math.cos are meant for radian measures!
        else:
            w_slope = 2.4 * m * g * v * math.sin(math.radians(slope)) * (0.3 ** (abs(slope) / 7.65))
        return w_level + w_slope


class explorerParameters:
    '''
	NOTE: This will not be used at all for BASALT/MINERVA. It is
	information that will be important for shadowing, which is likely
	beyond the scope of the project.
	
	This may eventually become incorporated into a different type of
	energy cost function, especially for the rover. Currently the optimization
	on energy only takes into account metabolic energy, and not energy
	gained or lost from the sun and shadowing.
	
	The explorer class should be designed such that this is optional
	and only necessary if we wish to perform shadowing
	
	The input p should be a dictionary of parameters
	'''

    def __init__(self, typeString, p=None):
        if typeString == 'Astronaut' and p == None:  # Default parameters for astronaut and rover
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
        elif typeString == 'Rover' and p == None:
            self.efficiency_SA = 0.26
            self.A_SA = 30
            self.batterySpecificEnergy = 145
            self.mBattery = 269
            self.electronicsPower = 1400
        elif typeString == 'Astronaut':  # There should be a dictionary of parameters given
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
        elif typeString == 'Rover':
            self.efficiency_SA = p['efficiency_SA']
            self.A_SA = p['A_SA']
            self.batterySpecificEnergy = p['batterySpecificEnergy']
            self.mBattery = p['mBattery']
            self.electronicsPower = p['electronicsPower']

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    slopes = np.linspace(-30, 30, 100)
    a = Astronaut(80)
    v = a.velocity(slopes)
    plt.plot(slopes, v)
    plt.show()