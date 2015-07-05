import numpy as np
import math
import utm
from osgeo import gdal, osr

class UTMCoord(object):
	'''
	Represents a UTM coordinate. All UTM Coordinates should be expressed in this form.
	'''
	def __init__(self, easting, northing, zone, zoneLetter = 'T'):
		self.easting = easting
		self.northing = northing
		self.zone = zone
		self.zoneLetter = zoneLetter # Note: 'T' is the zone letter of all but the northernmost point of Idaho
									 # 'Q' is the zone letter of Hawaii
									 
	def __str__(self):
		return "UTM Coordinate", self.easting, "E,",self.northing,"N, zone", self.zone, self.zoneLetter

class LatLongCoord(object):
	'''
	Represents a latitude and longitude pair. All Lat/Long Coordinates should be expressed in this form.
	'''
	def __init__(self, lat, long):
		self.latitude = lat
		self.longitude = long
	
	def __str__(self):
		return "LatLongCoord", self.latitude, ',', self.longitude
	
class EnvironmentalModel(object):
	'''
	This class ultimately represents an elevation map + all of the traversable spots on it.
	
	Public functions:
	
	setMaxSlope(slope) - sets the maximum slope that can be traversed
	setObstacle(coordinates), eraseObstacle(coordinates) - set or erase an obstacle at certain coordinates
	getElevation(coordinates), getSlope(coordinates) - get Elevation or Slope at certain coordinates
	isPassable(coordinates) - determines if a certain coordinate can be traversed
	convertToRowCol(coordinates), convertToUTM(coordinates), convertToLatLong(coordinates) - converts from one coordinate system to another
	loadElevationMap(fileName) - load an elevation map from a geoTIFF or text file
	'''
	def __init__(self, elevation_map, resolution, maxSlope, planet = "Earth", NW_Coord = UTMCoord(332107.99, 4692261.58, 19, 'T')):
		self.elevations = elevation_map #this is a numpy 2D array
		self.resolution = float(resolution) #this is just a float
		[gx, gy] = np.gradient(elevation_map, resolution, resolution)
		self.slopes = np.degrees(np.arctan(np.sqrt(np.add(np.square(gx),np.square(gy))))) # I think this syntax is correct
																						  # we want atan(sqrt(gx^2+gy^2)) in degrees
		self.obstacles = self.slopes <= maxSlope # obstacles is basically an "isPassable" function
		
		self.numRows = int(np.shape(elevation_map)[0])
		self.numCols = int(np.shape(elevation_map)[1]) # casted to int; doesn't make much sense to have them as longs
		self.planet = planet
		self.NW_UTM = self.convertToUTM(NW_Coord) # a UTMCoord object, default set to Boston

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
		# determines if coordinates can be passed through
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
		'''
		Converts a row/column or UTM coordinate into a LatLongCoord
		'''
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
		'''
		Converts a row/column coordinate or LatLongCoord into a UTMCoord.
		'''
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
			
	def loadElevationMap(self, file, maxSlope = 15, planet = 'Earth', NWCorner = None, SWCorner = None):
		'''
		Creates a EnvironmentalModel object from either a geoTiff file or a text file.
		
		Issue: computer sometimes freezes whenever loading a very large geoTIFF file (1GB+)

		Current modification: trying to enable loading a square sector of a geoTIFF file.
		2 new optional inputs: NWCorner and SE corner - UTM or LatLong coordinates of the subregion
		we would like to analyze.
		'''
		# file is a string representing the location of the file
		extension = file.split('.')[-1] # this should be the file extension
		
		if extension == 'txt':
			# This likely needs some updating
			f = open(file, r)
			inputs = [] #the values that end up in inputs will become numCols, numRows, xllcorner, yllcorner, cellsize, and NODATA_value
			for i in range(6):
				inputs.append(f.readline().split(' ')[-1]) # this should work given the current format
			numCols = inputs[0]
			numRows = inputs[1]
			mapArray = np.empty([numRows, numCols]) # initializes an empty array
			for i in range(numRows):
				x = f.readline().split(' ')
				if len(x) == numCols:
					mapArray[i] = x
				else:
					print "ERROR: expected " + str(numCols) + " columns. Got " + str(len(x)) + " columns"
					return 0
			return EnvironmentalModel(mapArray, inputs[4], maxSlope, planet)
		elif extension == 'tif':
			# NOTE: Currently, SEXTANT only supports geoTIFF files that use the UTM projection and have "north up"
			gdal.UseExceptions()
			
			# I don't actually understand how this code works I'm just copying people on stackexchange
			dataset = gdal.Open(file)
			band = dataset.GetRasterBand(1)
			proj = dataset.GetProjection()
			
			srs = osr.SpatialReference(wkt=proj)
			projcs = srs.GetAttrValue('projcs') # This will be a string that looks something like
												# "NAD83 / UTM zone 5N"
			
			if projcs: # projcs is not None for the government Hawaii data
				zone = projcs.split(' ')[-1][0:-1]
				zoneLetter = projcs.split(' ')[-1][-1]
				
			datasetInfo = dataset.GetGeoTransform()
			# returns a list of length 6. Indices 0 and 3 are the easting and northing values of the upper left corner.
			# Indices 1 and 5 are the w-e and n-s pixel resolutions, index 5 is always negative. Indicies 2 and 4 are
			# set to zero for all maps pointing in a "North up" type projection; for now we will only be using maps where
			# North is set to up.
			NWeasting = datasetInfo[0]
			NWnorthing = datasetInfo[3]
			if datasetInfo[1] == -datasetInfo[5]: # SEXTANT does not support maps where the x-resolution differs from the y-resolution at the moment
				resolution = datasetInfo[1]
			NWCoord = UTMCoord(NWeasting, NWnorthing, zone, zoneLetter)

			if NWCorner == None and SECorner == None: #No NW and SE corner implies we want the entire map
				mapArray = band.ReadAsArray() #converts from a raster band to a numpy array
				return EnvironmentalModel(mapArray, resolution, maxSlope, planet, NWCoord)
			else:
				top = self.convertToUTM(NWCorner.northing)
				bot = self.convertToUTM(SECorner.northing)
				left = self.convertToUTM(NWCorner.easting)
				right = self.convertToUTM(SECorner.easting)
				
				if bot > top or left > right:
					print "ERROR with NWCorner and SECorner"
					print "NWCorner: " + str(NWCorner) + " SWCorner: " + str(SECorner)
					return 0
				
				if left < NWeasting:
					left = NWeasting
				if right > NWeasting + datasetInfo[1] * (dataset.RasterXSize - 1):
					right = NWeasting + datasetInfo[1] * (dataset.RasterXSize - 1)
				if top > NWnorthing:
					top = NWnorthing
				if bot < NWnorthing + datasetInfo[5] * (dataset.RasterYSize - 1):
					bot = NWnorthing + datasetInfo[5] * (dataset.RasterYSize - 1)
				
				x_offset = int((left - NWeasting)/resolution)
				x_size = int((right - NWeasting)/resolution) + 1 - x_offset
				y_offset = int((NWnorthing - top)/resolution)
				y_size = ((NWnorthing - top)/resolution) + 1 - y_offset
				
				mapArray = band.ReadAsArray(x_offset, y_offset, x_size, y_size).astype(numpy.float)
				return EnvironmentalModel(mapArray, resolution, maxSlope, planet, NWCoord)
		else:
			print "ERROR: expected txt or tif file. Received " + extension + " type file"
			return 0