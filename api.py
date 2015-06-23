from EnvironmentalModel import UTMCoord, LatLongCoord, EnvironmentalModel
from ExplorationObjective import ActivityPoint
from ExplorerModel import Explorer, Rover, Astronaut

import heapq
import numpy as np
import math
import json
import utm

from osgeo import gdal, osr


class Pathfinder:
	'''
	This class performs the A* path finding algorithm and contains the Cost Functions. Also includes
	capabilities for analysis of a path.
	
	This class still needs performance testing for maps of larger sizes. I don't believe that
	we will be doing anything extremely computationally intensive though.
	
	Current cost functions are Time, Distance, and (Metabolic) Energy. It would be useful to be able to
	optimize on other resources like battery power or water sublimated, but those are significantly more
	difficult because they depend on shadowing and was not implemented by Aaron.
	'''
	def __init__(self, explorer_model, environmental_model, exploration_objectives):
		self.explorer = explorer_model
		self.map = environmental_model # a 2d array
		self.waypoints = exploration_objectives # this is a list of ActivityPoint objects
		self.numRows = environmental_model.numRows
		self.numCols = environmental_model.numCols
	
	def _goalTest(self, Node, endNode):
		'''
		Determines the criteria for "reaching" a node. Could potentially be modified to something like
		return distance < 5
		'''
		return Node.state == endNode.state # this should be the same no matter what value we are optimizing on
	
	def _costFunction(self, start_state, end_state, optimize_on):
		'''
		Given the start and end states, returns the cost of travelling between them.
		Allows for states which are not adjacent to each other.
		'''
		R = self.map.resolution
		startRow, startCol = start_state
		endRow, endCol = end_state
		startElev, endElev = self.map.getElevation(start_state), self.map.getElevation(end_state)
		
		path_length = R * math.sqrt((startRow-endRow)**2 + (startCol-endCol)**2)
		slope = math.degrees(math.atan(startElev - endElev) / path_length)
		
		if optimize_on == 'Distance':
			return self.explorer.distance(path_length)
		elif optimize_on == 'Time':
			return self.explorer.time(path_length, slope)
		elif optimize_on == 'Energy':
			return self.explorer.energyCost(path_length, slope)
		else:
			print "ERROR with optimize_on. Current value is " + optimize_on
			return None
	
	def _heuristic(self, state, endNode, optimize_on):
		'''
		A basic heuristic to speed up the search
		'''
		startRow, startCol = state
		endRow, endCol = endNode.state
		h_diagonal = min(abs(startRow-endRow), abs(startCol-endCol)) # max number of diagonal steps that can be taken
		h_straight = abs(startRow-endRow) + abs(startCol-endCol) # Manhattan distance
		if optimize_on == 'Energy':
			m = self.explorer.mass
			R = self.map.resolution
			if self.map.planet == 'Earth':
				D = (1.504 * m + 53.298) * R
			elif self.map.planet == 'Moon' and self.explorer.type == 'Astronaut':
				D = (2.295 * m + 52.936) * R
			elif self.map.planet == 'Moon' and self.explorer.type == 'Rover':
				P_e = self.explorer.P_e # this only exists for rovers
				D = (0.216 * m + P_e / 4.167) * R
			else:
				# This should not happen
				print "Error: something wrong with either the planet or the explorer"
				print "current planet: " + self.map.planet + "; current explorer type: " + self.explorer.type
				return 0
		elif optimize_on == 'Distance':
			D = self.map.resolution
		elif optimize_on == 'Time':
			D = self.map.resolution/(1.6) # the maximum velocity is 1.6 from Marquez 2008
		else:
			# Note: this should never actually happen
			print "Error: optimize_on is currently set to " + optimize_on
			return 0
		# Patel 2010. See page 49 of Aaron's thesis
		return D * math.sqrt(2) * h_diagonal + D * (h_straight - 2 * h_diagonal)
	
	def _getChildren(self, searchNode, optimize_on):
		# returns a list of states that can be travelled to
		# basically just the 8 surrounding tiles, if they are inbounds and not obstacles
		row = searchNode.state[0]
		col = searchNode.state[1]
		children = [(row+1, col),(row+1, col+1),(row+1, col-1),(row, col+1),(row, col-1),(row-1, col+1),(row-1, col),(row-1, col-1)]
		valid_children = [child for child in children if self.map.isPassable(child)]
		return [_SearchNode(state, searchNode, searchNode.cost + self._costFunction(searchNode.state, state, optimize_on)) for state in valid_children]
	
	def _aStarSearch(self, startNode, endNode, optimize_on):
		'''
		returns the path (as a list of coordinates), followed by the number of
		states expanded, followed by the total cost
	
		this function will return a list of states between two waypoints
		to get the full path between all waypoints, we will basically just use
		this function multiple times with each set
		as long as the number of waypoints is reasonable we should be fine
		
		costFunction can have three possible values: 'Energy', 'Time', or 'Distance'
		As of right now 'Energy' just refers to metabolic energy. It would be useful
		to calculate energy caused by shadowing but that is actually beyond the original
		functionalities of SEXTANT. A useful addition once most of the original
		capabilities are added
		'''
		if self._goalTest(startNode, endNode):
			return startNode.getPath()
		agenda = []
		heapq.heappush(agenda, (startNode.cost+self._heuristic(startNode.state, endNode, optimize_on), startNode)) #heapq requires
																						# the cost to come first I believe
		expanded = set()
		while len(agenda) > 0:
			priority, node = heapq.heappop(agenda)
			if node.state not in expanded:
				expanded.add(node.state)
				if self._goalTest(node, endNode):
					return (node.getPath(), len(expanded), node.cost)
				for child in self._getChildren(node, optimize_on):
					if child.state not in expanded:
						heapq.heappush(agenda,((child.cost+self._heuristic(child.state, endNode, optimize_on), child)))
		return (None, len(expanded), node.cost)

	def completePath(self, optimize_on):
		'''
		Returns a tuple representing the path and the total cost of the path.
		The path will be a list. All activity points will be duplicated in
		the returned path.
		'''
		finalPath = []
		totalCost = 0
		for i in range(len(self.waypoints)-1):
			node1 = _SearchNode(self.waypoints[i].coordinates, None, 0)
			node2 = _SearchNode(self.waypoints[i+1].coordinates, None, 0)
			path, expanded, cost = self._aStarSearch(node1, node2, optimize_on)
			
			finalPath += path # for now I'm not going to bother with deleting the duplicates
							  # that will occur at every activity point. I think it might end up being useful
			totalCost += cost
			if optimize_on == 'Time':
				totalCost += self.wayPoints[i].duration # the final waypoint shouldn't have any duration, so this should be fine
		return (finalPath, totalCost)
	
	def toJSON(self, optimize_on):
		'''
		This returns the list following "sequence." It doesn't really make sense
		for SEXTANT to actually do parts of the JSON file such as "creator."
		'''
		path = self.completePath(optimize_on)[0]
		activityPoints = self.waypoints
		
		sequence = []
		
		def nextStation():
			AP = activityPoints.pop(0)
			element = AP.information
			
			latLong = self.map.convertToLatLong(AP.coordinates)
			
			element["geometry"] = {"coordinates": [latLong.latitude, latLong.longitude], "type": "Point"}
			element["type"] = "Station"
			return element
		
		for i, point in enumerate(path):
			# first set of if/elif statements creates the "Station" or "pathPoint" things,
			# second set of if/elif statements creates the "segments" connecting them
			if i == 0 or i == len(path) - 1: # first and last elements are always station
				sequence.append(nextStation())
			elif point == path[i-1]: # point is identical to the previous one and is thus a station
				sequence.append(nextStation())
			else: # This point is not a station and is thus a "pathPoint," basically just a point inside the path
				latLongCoords = [self.map.convertToLatLong(point).latitude, self.map.convertToLatLong(point).longitude]
				sequence.append({"geometry": {"coordinates": latLongCoords, "type": "Point"}})
				
			if i == (len(path)-1): # We have reached the end of the path and therefore have no more segments
				pass
			elif path[i] == path[i+1]: # This indicates the first coordinate in a station
				pass
			else: # Otherwise we need to add a segment
				startState, endState = path[i], path[i+1]
				distance = self._costFunction(startState, endState, "Distance")
				energy = self._costFunction(startState, endState, "Energy")
				time = self._costFunction(startState, endState, "Time")
				
				velocity = distance/time
				energyRate = energy/time
				
				element = {"distance": distance,
						   "energy": energy,
						   "energyRate": energyRate,
						   "time": time,
						   "type": "Segment",
						   "velocity": velocity}
				sequence.append(element)
		return sequence
	
	def analysePath(self, path, factor = 'Energy'):
		#coordPath = self.map.latLongToCoordinates(path)
		coordPath = path
		data = [0]
		
		for i in range(len(coordPath)-1):
			pair = (coordPath[i], coordPath[i+1])
			if pair[0] != pair[1]:
				if factor == 'Energy':
					data.append(self._costFunction(coordPath[i], coordPath[i+1], 'Energy'))
				elif factor == 'Time':
					data.append(self._costFunction(coordPath[i], coordPath[i+1], 'Time'))
				elif factor == 'Distance':
					data.append(self._costFunction(coordPath[i], coordPath[i+1], 'Distance'))
				elif factor == 'CumulativeEnergy':
					data.append(self._costFunction(coordPath[i], coordPath[i+1], 'Energy') + data[-1])
				elif factor == 'CumulativeTime':
					data.append(self._costFunction(coordPath[i], coordPath[i+1], 'Time') + data[-1])
				elif factor == 'CumulativeDistance':
					data.append(self._costFunction(coordPath[i], coordPath[i+1], 'Distance') + data[-1])
				else:
					print "Error: unknown factor " +repr(factor) + " in analyzePath"
		return data

class _SearchNode:
	'''
	This class is only used to assist in the A* search
	'''
	def __init__(self,state,parent,cost = 0):
		self.state = state
		self.parent = parent
		self.cost = cost
		
	def getPath(self):
		path = [self.state]
		node = self
		while node.parent:
			path.append(node.parent.state)
			node = node.parent
			
		path.reverse()
		return path

def loadElevationMap(file, maxSlope = 15, planet = 'Earth'):
	'''
	Creates a EnvironmentalModel object from either a geoTiff file or a text file.
	I'm not sure if the geoTiff file includes information about the resolution which
	might be a bit of a problem...
	
	Current issue: computer is freezing whenever I try to load a very large geoTiff file.
	Actually I think it's fine as long as you don't try to load it directly from command line
	'''
	# file is a string representing the location of the file
	extension = file.split('.')[-1] # this should be the file extension
	
	if extension == 'txt':
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
		gdal.UseExceptions()
		
		# I don't actually understand how this code works I'm just copying people on stackexchange
		dataset = gdal.Open(file)
		band = dataset.GetRasterBand(1)
		mapArray = band.ReadAsArray()
		proj = dataset.GetProjection()
		
		srs = osr.SpatialReference(wkt=proj)
		projcs = srs.GetAttrValue('projcs') # This will be a string that looks something like
											# "NAD83 / UTM zone 5N"
		
		zone = projcs.split(' ')[-1][0:-1]
		zoneLetter = projcs.split(' ')[-1][-1]
		
		print projcs
		print zone, zoneLetter
		
		datasetInfo = dataset.GetGeoTransform()
		# returns a list of length 6. Indices 0 and 3 are the easting and northing values of the upper left corner.
		# Indices 1 and 5 are the w-e and n-s pixel resolutions, index 5 is always negative. Indicies 2 and 4 are
		# set to zero for all maps pointing in a "North is up" type projection
		easting = datasetInfo[0]
		northing = datasetInfo[3]
		if datasetInfo[1] == -datasetInfo[5]:
			resolution = datasetInfo[1]
		
		return EnvironmentalModel(mapArray, resolution, maxSlope, planet, UTMCoord(easting, northing, zone, zoneLetter))
	else:
		print "ERROR: expected txt or tif file. Received " + extension + " type file"
		return 0