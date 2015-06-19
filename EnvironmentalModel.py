import numpy as np
import math
import utm

class UTMCoord:
	def __init__(self, easting, northing, zone, zoneLetter = 'T'):
		self.easting = easting
		self.northing = northing
		self.zone = zone
		self.zoneLetter = zoneLetter # Note: 'T' is the zone letter of all but the northernmost point of Idaho
									 # 'Q' is the zone letter of Hawaii

class LatLongCoord:
	def __init__(self, lat, long):
		self.latitude = lat
		self.longitude = long
		
class EnvironmentalModel:
	def __init__(self, elevation_map, resolution, maxSlope, planet = "Earth", NW_UTM = UTMCoord(332107.99, 4692261.58, 19, 'T')):
		self.elevations = elevation_map #this is a numpy 2D array
		self.resolution = float(resolution) #this is just a float
		[gx, gy] = np.gradient(elevation_map, resolution, resolution)
		self.slopes = np.degrees(np.arctan(np.sqrt(np.add(np.square(gx),np.square(gy))))) # I think this syntax is correct
																						  # we want atan(sqrt(gx^2+gy^2)) in degrees
		self.obstacles = self.slopes <= maxSlope # obstacles is basically an "isPassable" function
		
		self.numRows = int(np.shape(elevation_map)[0])
		self.numCols = int(np.shape(elevation_map)[1]) # casted to int; doesn't make much sense to have them as longs
		self.planet = planet
		self.NW_UTM = NW_UTM # a UTMCoord object, default set to Boston

	def setMaxSlope(self, maxSlope):
		self.obstacles = self.slopes <= maxSlope
	
	def setObstacle(self, coordinates):
		row, col = self.convertToRowCol(coordinates) # of the form (row, column)
		self.obstacles[row][col] = False
	
	def eraseObstacle(self, coordinates):
		row, col = self.convertToRowCol(coordinates)
		self.obstacles[row][col] = True
	
	def getElevation(self, coordinates):
		row, col = self.convertToRowCol(coordinates)
		return self.elevations[row][col]
		
	def getSlope(self, coordinates):
		row, col = self.convertToRowCol(coordinates)
		return self.slopes[row][col]
	
	def _inBounds(self, state):
		# determines if a state is within the boundaries of the environmental model
		# a state is a tuple of the form (row, column)
		row = state[0]
		col = state[1]
		return (row in range(self.numRows)) and (col in range(self.numCols))
	
	def isPassable(self, coordinates):
		row, col = self.convertToRowCol(coordinates) # coordinates is a tuple
		if not self._inBounds(coordinates):
			return False
		else:
			return self.obstacles[row][col]
	
	def _UTMtoRowCol(self, UTM):
		'''
		Converts from UTMCoord object to a tuple representing the row and the column
		in the matrix. There may be minor rounding issues
		
		As of right now both UTMtoRowCol and RowColToUTM only work if we do not cross a UTM line.
		Eventually for completeness sake this should probably be changed
		'''
		if UTM.zone == self.NW_UTM.zone:
			row = round((self.NW_UTM.y - UTM.y)/self.resolution)
			col = round((UTM.x - self.NW_UTM.x)/self.resolution)
			return row, col
		else:
			print "WARNING: map is in UTM zone " + repr(self.NW_UTM.zone)
			print "Zone given: " + repr(UTM.zone)
			pass # Later I could figure out how to account for zone changes just in case a site is
				 # just on the border of a UTM zone.
	
	def _rowColToUTM(self, coordinates):
		'''
		The reverse of the above function.
		'''
		row, col = coordinates
		
		xNW, yNW = self.NW_UTM.easting, self.NW_UTM.northing
		x = xNW + col * self.resolution
		y = yNW - row * self.resolution
		return UTMCoord(x, y, self.NW_UTM.zone, self.NW_UTM.zoneLetter)
		
	def convertToRowCol(self, position):
		"""
		This should be called before every function in EnvironmentalModel. Converts UTMCoord or LatLongCoord
		objects into a tuple representing the row and the column of the environmental map closest to that location.
		"""
		if type(position) is UTMCoord:
			return self._UTMtoRowCol(position)
		elif type(position) is LatLongCoord:
			easting, northing, zoneNumber, zoneLetter = utm.from_latlon(position.latitude, position.longitude)
			return self._UTMtoRowCol(UTMCoord(easting, northing, zoneNumber, zoneLetter))
		elif type(position) is tuple:
			return position
		else:
			print "ERROR: only accepts UTMCoord, LatLongCoord, and tuple objects"
			print "Received " + repr(type(position)) + " object"
			return 0
	
	def convertToLatLong(self, position):
		if type(position) is UTMCoord:
			easting, northing, zoneNumber, zoneLetter = position.easting, position.northing, position.zone, position.zoneLetter
			lat, long = utm.to_latlon(easting, northing, zoneNumber, zoneLetter)
			return LatLongCoord(lat, long)
		elif type(position) is LatLongCoord:
			return position
		elif type(position) is tuple:
			UTM = self._rowColToUTM(position)
			easting, northing, zoneNumber, zoneLetter = UTM.easting, UTM.northing, UTM.zone, UTM.zoneLetter
			lat, long = utm.to_latlon(easting, northing, zoneNumber, zoneLetter)
			return LatLongCoord(lat, long)
		else:
			print "ERROR: only accepts UTMCoord, LatLongCoord, and tuple objects"
			print "Received " + repr(type(position)) + " object"
			return 0
	
	def convertToUTM(self, position):
		if type(position) is UTMCoord:
			return position
		elif type(position) is LatLongCoord:
			easting, northing, zoneNumber, zoneLetter = utm.fromLatLong(position.latitude, position.longitude)
			return UTMCoord(easting, northing, zoneNumber, zoneHemisphere)
		elif type(position) is tuple:
			return self._rowColToUTM(position)
		else:
			print "ERROR: only accepts UTMCoord, LatLongCoord, and tuple objects"
			print "Received " + repr(type(position)) + " object"
			return 0