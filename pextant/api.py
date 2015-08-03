from EnvironmentalModel import UTMCoord, LatLongCoord, EnvironmentalModel
from ExplorationObjective import ActivityPoint
from ExplorerModel import Explorer, Rover, Astronaut

import csv
import heapq
import numpy as np
import math
import json
from scipy.optimize import minimize

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
	def __init__(self, explorer_model, environmental_model):
		self.explorer = explorer_model
		self.map = environmental_model # a 2d array
		
	def _goalTest(self, Node, endNode):
		'''
		Determines the criteria for "reaching" a node. Could potentially be modified to something like
		return distance < 5
		'''
		return Node.state == endNode.state # this should be the same no matter what value we are optimizing on
	
	def _aStarCostFunction(self, start_state, end_state, optimize_vector):
		'''
		Given the start and end states, returns the cost of travelling between them.
		Allows for states which are not adjacent to each other.
		
		Note that this uses start and end coordinates rather than nodes.
		
		optimize_vector is a list or tuple of length 3, representing the weights of
		Distance, Time, and Energy
		'''
		R = self.map.resolution
		startRow, startCol = start_state
		endRow, endCol = end_state
		startElev, endElev = self.map.getElevation(start_state), self.map.getElevation(end_state)
		
		path_length = R * math.sqrt((startRow-endRow)**2 + (startCol-endCol)**2)
		slope = math.degrees(math.atan((startElev - endElev) / path_length))
		
		distWeight = self.explorer.distance(path_length)*optimize_vector[0]
		timeWeight = self.explorer.time(path_length, slope)*optimize_vector[1]
		energyWeight = self.explorer.energyCost(path_length, slope, self.map.getGravity())*optimize_vector[2]
		
		return distWeight + timeWeight + energyWeight
	
	def _heuristic(self, currentNode, endNode, optimize_on, search_algorithm):
		'''
		A basic heuristic to speed up the search
		'''
		startRow, startCol = currentNode.state
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
		if search_algorithm == "A*":
			# Patel 2010. See page 49 of Aaron's thesis
			return D * math.sqrt(2) * h_diagonal + D * (h_straight - 2 * h_diagonal)
		elif search_algorithm == "Field D*":
			# This is just Euclidean distance
			return D * math.sqrt((startRow - endRow)**2 + (startCol - endCol)**2)
	
	def _aStarGetNeighbors(self, searchNode, optimize_on):
		# returns a list of states that can be travelled to
		# basically just the 8 surrounding tiles, if they are inbounds and not obstacles
		row = searchNode.state[0]
		col = searchNode.state[1]
		children = [(row+1, col),(row+1, col+1),(row+1, col-1),(row, col+1),(row, col-1),(row-1, col+1),(row-1, col),(row-1, col-1)]
		valid_children = [child for child in children if self.map.isPassable(child)]
		return [_aStarSearchNode(state, searchNode, searchNode.cost + self._aStarCostFunction(searchNode.state, state, optimize_on)) for state in valid_children]
	
	def aStarSearch(self, startNode, endNode, optimize_on):
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
		heapq.heappush(agenda, (startNode.cost+self._heuristic(startNode, endNode, optimize_on, "A*"), startNode)) #heapq requires
																						# the cost to come first I believe
		expanded = set()
		while len(agenda) > 0:
			priority, node = heapq.heappop(agenda)
			if node.state not in expanded:
				expanded.add(node.state)
				if self._goalTest(node, endNode):
					return (node.getPath(), len(expanded), node.cost)
				for child in self._aStarGetNeighbors(node, optimize_on):
					if child.state not in expanded:
						heapq.heappush(agenda,((child.cost+self._heuristic(child, endNode, optimize_on, "A*"), child)))
		return (None, len(expanded), node.cost)

	def completePath(self, optimize_on, waypoints, returnType = "JSON", fileName = None):
		'''
		Returns a tuple representing the path and the total cost of the path.
		The path will be a list. All activity points will be duplicated in
		the returned path.
		
		waypoints is a list of activityPoint objects, in the correct order. fileName is
		used when we would like to write stuff to a file and is currently necessary
		for csv return types.
		'''
		finalPath = []
		costs = []
		for i in range(len(waypoints)-1):
			segmentCost = 0
		
			node1 = _aStarSearchNode(self.map.convertToRowCol(waypoints[i].coordinates), None, 0)
			node2 = _aStarSearchNode(self.map.convertToRowCol(waypoints[i+1].coordinates), None, 0)
			path, expanded, cost = self.aStarSearch(node1, node2, optimize_on)
			
			finalPath += path # for now I'm not going to bother with deleting the duplicates
							  # that will occur at every activity point. I think it might end up being useful
			segmentCost += cost
			if optimize_on == 'Time':
				segmentCost += wayPoints[i].duration # the final waypoint shouldn't have any duration, so this should be fine
			
			costs.append(segmentCost)
		if returnType == "tuple":
			return (finalPath, costs, sum(costs))
		elif returnType == "JSON":
			data = self._toJSON(finalPath, optimize_on, waypoints)
			if fileName:
				with open('data.json', 'w') as outfile:
					json.dump(data, outfile, indent = 4)
			return data
		elif returnType == "csv":
			sequence = self._toCSV(finalPath, optimize_on, waypoints)
			print sequence
			if fileName:
				with open(fileName, 'wb') as csvfile:
					writer = csv.writer(csvfile)
					for row in sequence:
						writer.writerow(row)
			return sequence
		
#########################################################################################
# Above is A*; below is field D* (see Ferguson and Stentz 2005)							#
#########################################################################################
		
	def _fieldDStarGetNeighbors(self, searchNode):
		row = searchNode.state[0]
		col = searchNode.state[1]
		children = [(row+1, col),(row+1, col+1),(row+1, col-1),(row, col+1),(row, col-1),(row-1, col+1),(row-1, col),(row-1, col-1)]
		valid_children = [child for child in children if self.map.isPassable(child)]
		return valid_children
		
	def _fieldDStarGetConsecutiveNeighbors(self, searchNode):
		'''
		To calculate cost, field D* requires pairs of consecutive neighbors
		
		Note that neighbors are returned as tuples, not _FieldDStarSearchNode objects
		'''
		row = searchNode.state[0]
		col = searchNode.state[1]
	
		consecutive_neighbors = [((row+1, col), (row+1, col+1)),
								 ((row+1, col+1), (row, col+1)),
								 ((row, col+1), (row-1, col+1)),
								 ((row-1, col+1), (row-1, col)),
								 ((row-1, col), (row-1, col-1)),
								 ((row-1, col-1), (row, col-1)),
								 ((row, col-1), (row+1, col-1)),
								 ((row+1, col-1), (row+1, col))]
		valid_consecutive_neighbors = [item for item in consecutive_neighbors if (self.map._inBounds(item[0]) and self.map._inBounds(item[1]))]
		return valid_consecutive_neighbors
		
	def _fieldDStarComputeCost(self, node, neighbor_a, neighbor_b, optimize_on, numTestPoints): #neighbor_a and neighbor_b are nodes, not coordinates
		# Check the documentation for more information about the Compute Cost function
		R = self.map.resolution
		row, col = node.state
		
		# s_1 is the horizontal neighbor, s_2 is the diagonal neighbor
		if neighbor_a.state[0] == row or neighbor_a.state[1] == col:
			s_1 = neighbor_a
			s_2 = neighbor_b
		else:
			s_1 = neighbor_b
			s_2 = neighbor_a
		
		if optimize_on == 'Distance':
		# following the algorithm from Ferguson and Stentz
			
			c = self.explorer.distance(R*math.sqrt(2))
			b = self.explorer.distance(R)		

			if min(c, b) == float('inf'):
				return float('inf')
			elif s_1.cost <= s_2.cost:
				return min(c, b) + s_1.cost
			else:
				f = s_1.cost - s_2.cost
				if f <= b:
					if c <= f:
						return s_2.cost + c * math.sqrt(2)
					else:
						y = min(f/math.sqrt(c**2 - f**2), 1)
						return c * math.sqrt(1 + y**2)+ f*(1-y) + s2.cost
				else:
					if c <= b:
						return c * math.sqrt(2) + s_2.cost
					else:
						x = 1 - min(b/math.sqrt(c**2-b**2), 1)
						return c * math.sqrt(1+(1-x)**2) + b*x + s_2.cost
		elif optimize_on == 'Time' or 'Energy':
		# we can't use Ferguson and Stentz here
			c_1 = s_1.cost
			h_1 = self.map.getElevation(s_1.state)
			c_2 = s_2.cost
			h_2 = self.map.getElevation(s_2.state)
			h = self.map.getElevation(node.state)
			
			def function1(x):
			# This is the function which goes horizontally first, then directly to the corner
				prevCost = c_2
				height = h_1*x + h*(1-x)
				dist1 = x*R
				slope1 = math.degrees(math.atan((h-height)/dist1))
				dist2 = math.sqrt(1+x**2) * R
				slope2 = math.degrees(math.atan((height - h_2)/dist2))
				
				if optimize_on == 'Time':
					return prevCost + self.explorer.time(dist1, slope1) + self.explorer.time(dist2, slope2)
				elif optimize_on == 'Energy':
					return prevCost + self.explorer.energyCost(dist1, slope1) + self.explorer.energyCost(dist2, slope2)
				
			def function2(y):
			# This is the function which goes directly to the opposite side
			# y represents the y-coordinate of the side and is a number between 0 and 1
				prevCost = y*c_2 + (1-y)*c_1
				height = y*h_2 + (1-y)*h_1
				dist = math.sqrt(1+y**2)*R
				slope = math.degrees(math.atan((h - height) / (dist)))
				
				if optimize_on == 'Time':
					return prevCost + self.explorer.time(dist, slope)
				elif optimize_on == 'Energy':
					return prevCost + self.explorer.energyCost(dist, slope)
			
			step = 1.0/numTestPoints
			# evenly spread test points
			testPoints = [step/2 + step * i for i in range(numTestPoints)]
			
			# We have several test points to determine our starting location
			func1Points = [function1(tp) for tp in testPoints]
			func2Points = [function2(tp) for tp in testPoints]
			startPoint1 = testPoints[np.argmin(func1Points)]
			startPoint2 = testPoints[np.argmin(func2Points)]
			
			# Not too sure if SLSPQ is the best choice. I chose it because it allows me to set bounds.
			min1 = minimize(function1, startPoint1, method = 'SLSPQ', bounds = ((0, 1))).x
			min2 = minimize(function2, startPoint2, method = 'SLSPQ', bounds = ((0, 1))).x
			
			return min(min1, min2)
	
	def _fieldDStarGetKey(self, node, start_node, optimize_on):
		return (min(node.cost, node.rhs) + self._heuristic(node, start_node, optimize_on, "Field D*"), min(node.cost, node.rhs))
	
	def fieldDStarSearch(self, startNode, endNode, optimize_on, numTestPoints = 10):
		startNode.cost = float('inf')
		startNode.rhs = float('inf')
		endNode.cost = float('inf')
		endNode.rhs = 0
		
		open = []
		expanded = set()
		
		def updateState(node):
			open_nodes = [pair[1] for pair in open]
			# If node was never previously visited, cost = infinity
			if node not in expanded:
				node.cost = float('inf')
			# If node != goal
			# rhs(node) = min_{(s', s'') in connbrs(node)} ComputeCost(node, s', s'')
			if not self._goalTest(node, endNode):
				rhs = float('inf')
				for pair in self._fieldDStarGetConsecutiveNeighbors(node):
					neighbor_a, neighbor_b = pair
					test_val = self._fieldDStarComputeCost(node, neighbor_a, neighbor_b)
					if test_val < rhs:
						rhs = test_val
			# if node in open, remove node from open
			if node in open_nodes:
				for pair in open:
					if pair[1] == node:
						open.remove(pair)
			# if cost != rhs, insert s into open with key(node)
			if node.cost != node.rhs:
				heapq.heappush(open, (_fieldDStarGetKey(node, startNode, optimize_on)))
		
		def computeShortestPath():
			while startNode.cost != startNode.rhs:
				key, node = heapq.heappop(open)
			# Add more stuff here later
		
	def _toJSON(self, path, optimize_on, waypoints):
		'''
		This returns the list following "sequence." It doesn't really make sense
		for SEXTANT to actually do parts of the JSON file such as "creator."
		'''		
		sequence = []
		
		def nextStation():
			AP = waypoints.pop(0)
			element = {}
			
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
				distance = self._aStarCostFunction(startState, endState, [1, 0, 0]) # distance
				energy = self._aStarCostFunction(startState, endState, [0, 0, 1]) # energy
				time = self._aStarCostFunction(startState, endState, [0, 1, 0]) # time
				
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
		
	def _toCSV(self, path, optimize_on, waypoints):
		
		sequence = []
		
		def nextStation():
			AP = waypoints.pop(0)
			
			latLong = self.map.convertToLatLong(AP.coordinates)
			elev = self.map.getElevation(AP.coordinates)
			
			return [True, latLong.longitude, latLong.latitude, elev]
		
		for i, point in enumerate(path):
			row = []
			# first set of if/elif statements creates the "Station" or "pathPoint" things,
			# second set of if/elif statements creates the "segments" connecting them
			if i == 0 or i == len(path) - 1: # first and last elements are always station
				row += nextStation()
			elif point == path[i+1]: # point is identical to the next one and is thus a station
				pass
			elif point == path[i-1]:
				row += nextStation()
			else: # This point is not a station and is thus a "pathPoint," basically just a point inside the path
				latLongCoords = [self.map.convertToLatLong(point).longitude, self.map.convertToLatLong(point).latitude]
				elev = self.map.getElevation(point)
				row += ([False] + latLongCoords + [elev])
				
			if i == (len(path)-1): # We have reached the end of the path
				pass
			elif path[i] == path[i+1]: # This indicates the first coordinate in a station
				pass # We add nothing to the csv
			else: # Otherwise we need to add a segment
				startState, endState = path[i], path[i+1]
				distance = self._aStarCostFunction(startState, endState, [1, 0, 0])
				energy = self._aStarCostFunction(startState, endState, [0, 0, 1])
				time = self._aStarCostFunction(startState, endState, [1, 0, 1])

				row += [distance, energy, time]
				
			if (i == (len(path)-1)) or (path[i] != path[i+1]):
				sequence += [row]
				
		sequence = [['isStation', 'x', 'y', 'z', 'distanceMeters', 'energyJoules', 'timeSeconds']] + sequence
		
		return sequence
	
class _aStarSearchNode:
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

class _FieldDStarNode:
	'''
	This class is intended to assist for the DStar search mode.
	This takes longer but is much more flexible, as it allows
	for more than the 8 cardinal directions.
	'''
	def __init__(self,state,parent,cost = 0,rhs = float('inf')):
		self.state = state
		self.parent = parent
		self.cost = cost
		self.rhs = rhs # default set to positive infinity
	
	def getPath(self):
		path = [self.state]
		node = self
		while node.parent:
			path.append(node.parent.state)
			node = node.parent
			
		path.reverse()
		return path