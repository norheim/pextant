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
	
	def _vectorize(self, optimize_on):
		if optimize_on == "Energy":
			return [0, 0, 1]
		elif optimize_on == "Time":
			return [0, 1, 0]
		elif optimize_on == "Distance":
			return [1, 0, 0]
		else:
			return optimize_on
	
	def _aStarCostFunction(self, start_state, end_state, optimize_on):
		'''
		Given the start and end states, returns the cost of travelling between them.
		Allows for states which are not adjacent to each other.
		
		Note that this uses start and end coordinates rather than nodes.
		
		optimize_vector is a list or tuple of length 3, representing the weights of
		Distance, Time, and Energy
		'''
		optimize_vector = self._vectorize(optimize_on)
		
		R = self.map.resolution
		startRow, startCol = start_state
		endRow, endCol = end_state
		startElev, endElev = self.map.getElevation(start_state), self.map.getElevation(end_state)
		
		path_length = R * math.sqrt((startRow-endRow)**2 + (startCol-endCol)**2)
		slope = math.degrees(math.atan((endElev - startElev) / path_length))
				
		distWeight = self.explorer.distance(path_length)*optimize_vector[0]
		timeWeight = self.explorer.time(path_length, slope)*optimize_vector[1]
		energyWeight = self.explorer.energyCost(path_length, slope, self.map.getGravity())*optimize_vector[2]
		
		return distWeight + timeWeight + energyWeight
	
	def _heuristic(self, currentNode, endNode, optimize_on, search_algorithm):
		'''
		A basic heuristic to speed up the search
		'''
		startRow, startCol = currentNode.coordinates
		endRow, endCol = endNode.coordinates
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
			# Note: this means we have a vector optimize_on
			energyCost = optimize_on[2]*self._heuristic(currentNode, endNode, 'Energy', search_algorithm)
			timeCost = optimize_on[1]*self._heuristic(currentNode, endNode, 'Time', search_algorithm)
			distanceCost = optimize_on[0]*self._heuristic(currentNode, endNode, 'Distance', search_algorithm)
			return energyCost + timeCost + distanceCost
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
		return [aStarSearchNode(state, searchNode, searchNode.cost + \
			self._aStarCostFunction(searchNode.state, state, optimize_on)) for state in valid_children]
	
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
		optimize_vector = self._vectorize(optimize_on)
		
		finalPath = []
		costs = []
		for i in range(len(waypoints)-1):
			segmentCost = 0
		
			node1 = aStarSearchNode(self.map.convertToRowCol(waypoints[i].coordinates), None, 0)
			node2 = aStarSearchNode(self.map.convertToRowCol(waypoints[i+1].coordinates), None, 0)
			path, expanded, cost = self.aStarSearch(node1, node2, optimize_vector)
			
			finalPath += path # for now I'm not going to bother with deleting the duplicates
							  # that will occur at every activity point. I think it might end up being useful
			segmentCost += cost
			segmentCost += optimize_vector[1]*waypoints[i].duration
			
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
		
	def _fieldDStarGetConsecutiveNeighbors(self, coordinates):
		'''
		To calculate cost, field D* requires pairs of consecutive neighbors
		
		Note that neighbors are returned as tuples
		'''
		row = coordinates[0]
		col = coordinates[1]
	
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
		
	def _fieldDStarComputeCost(self, node, neighbor_a, neighbor_b, optimize_on, nodeList = None, numTestPoints = 10):
		# neighbor_a and neighbor_b must be nodes, not coordinates
		# This function returns a tuple - the point on the edge that is intersected, and the cost
		# Check the documentation for more information about the Compute Cost function
		R = self.map.resolution
		
		row, col = node.coordinates
		# s_1 is the horizontal neighbor, s_2 is the diagonal neighbor
		if neighbor_a.coordinates[0] == row or neighbor_a.coordinates[1] == col:
			s_1 = neighbor_a
			s_2 = neighbor_b
		else:
			s_1 = neighbor_b
			s_2 = neighbor_a
		
		if optimize_on == 'Distance':
		# Adapted from Ferguson and Stentz I think it should be correct...
		
			b = self.explorer.distance(R)		

			if b == float('inf'):
				return float('inf')
			elif s_1.cost <= s_2.cost:
				return (s_1.state, b + s_1.cost)
			else:
				f = s_1.cost - s_2.cost
				if f < b:
					y = min(f/math.sqrt(b**2 - f**2), 1)
					point = ((1-y)*s_1.state[0] + y*s_2.state[0], (1-y)*s_1.state[1] + y*s_2.state[1])
					return (point, b * math.sqrt(1 + y**2)+ f*(1-y) + s_2.cost)
				else:
					return (s_2.state, b * math.sqrt(2) + s_2.cost)
		else:
		# we have either "Energy", "Time", or a vector costfunction
		# we can't use Ferguson and Stentz here
			c_1 = s_1.cost
			h_1 = self.map.getElevation(s_1.coordinates)
			c_2 = s_2.cost
			h_2 = self.map.getElevation(s_2.coordinates)
			h = self.map.getElevation(node.coordinates)
			
			# This is necessary for "normal" Field D* but in our case I don't think it makes
			# a lot of sense. So for now, everything having to do with function 1 is commented out
			
			'''
			def function1(x):
			# This is the function which goes horizontally first, then directly to the corner
				prevCost = c_2
				height = h_1*x + h*(1-x)
				dist1 = x*R
				slope1 = math.degrees(math.atan((height - h)/dist1))
				dist2 = math.sqrt(1+x**2) * R
				slope2 = math.degrees(math.atan((h_2 - height)/dist2))
				
				if optimize_on == 'Time':
					return prevCost + self.explorer.time(dist1, slope1) + self.explorer.time(dist2, slope2)
				elif optimize_on == 'Energy':
					return prevCost + self.explorer.energyCost(dist1, slope1, self.map.getGravity()) + \
							self.explorer.energyCost(dist2, slope2, self.map.getGravity())
				else:
					d = self.explorer.distance(dist1) + self.explorer.distance(dist2)
					t = self.explorer.time(dist1, slope1) + self.explorer.time(dist2, slope2)
					e = self.explorer.energyCost(dist1, slope1, self.map.getGravity()) + \
							self.explorer.energyCost(dist2, slope2, self.map.getGravity())
					return prevCost + d*optimize_on[0] + t*optimize_on[1] + e*optimize_on[2]
			'''
			
			# This is the function which goes directly to the opposite side
			# y represents the y-coordinate of the side and is a number between 0 and 1
			
			def function2(y):
				prevCost = y*c_2 + (1-y)*c_1
				height = y*h_2 + (1-y)*h_1
				dist = math.sqrt(1+y**2)*R
				slope = math.degrees(math.atan((height - h) / (dist)))
								
				if optimize_on == 'Time':
					return prevCost + self.explorer.time(dist, slope)
				elif optimize_on == 'Energy':
					return prevCost + self.explorer.energyCost(dist, slope, self.map.getGravity())
				else:
					d = self.explorer.distance(dist)
					t = self.explorer.time(dist, slope)
					e = self.explorer.energyCost(dist, slope, self.map.getGravity())
					return d*optimize_on[0] + t*optimize_on[1] + e*optimize_on[2]
			
			step = 1.0/numTestPoints
			# evenly spread test points
			testPoints = [step/2 + step * i for i in range(numTestPoints)]
			
			# We have several test points to determine our starting location
			# func1Points = [function1(tp) for tp in testPoints]
			# startPoint1 = testPoints[np.argmin(func1Points)]
			func2Points = [function2(tp) for tp in testPoints]
			startPoint2 = testPoints[np.argmin(func2Points)]
			
			# Not too sure if SLSQP is the best choice. I chose it because it allows me to set bounds.
			# We choose the smaller of the two functions
			#min1 = minimize(function1, startPoint1, method = 'SLSQP', bounds = ((0, 1))).x
			min2 = float(minimize(function2, startPoint2, method = 'SLSQP', bounds = [(0, 1)]).x)
			
			# point is the point that is corresponds by the minimum value
			point = ((1-min2)*s_1.coordinates[0] + min2*s_2.coordinates[0], (1-min2)*s_1.coordinates[1] + min2*s_2.coordinates[1])
			
			return (point, function2(min2))
				
	def _fieldDStarGetKey(self, node, start_node, optimize_on):
		return (min(node.cost, node.rhs) + self._heuristic(node, start_node, optimize_on, "Field D*"), min(node.cost, node.rhs))
		
	def _fieldDStarUpdateState(nodeDict, open, coordinates):
		# If node was never previously visited, cost = infinity
		if coordinates not in nodeDict.keys():
			node = FieldDStarNode(coordinates, float('inf'), float('inf'))
		else:
			node = nodeDict[coordinates]
		
		# If node != goal
		# rhs(node) = min_{(s', s'') in connbrs(node)} ComputeCost(node, s', s'')
		if not self._goalTest(node, endNode):
			rhs = float('inf')
			for pair in self._fieldDStarGetConsecutiveNeighbors(node):
				neighbor_a, neighbor_b = pair
				test_val = self._fieldDStarComputeCost(node, neighbor_a, neighbor_b)[1]
				if test_val < rhs:
					rhs = test_val
			node.rhs = rhs
		
		# updating nodeDict
		nodeDict[coordinates] = node
		
		# if node in open, remove node from open
		open_nodes = [pair[1] for pair in open]
		if node in open_nodes:
			for pair in open:
				if pair[1] == node:
					open.remove(pair)
		
		# if cost != rhs, insert node into open with key(node)
		if node.cost != node.rhs:
			heapq.heappush(open, (_fieldDStarGetKey(node, startNode, optimize_on)))
	
	def _fieldDStarComputeShortestPath(self, startNode, open):
		while startNode.cost != startNode.rhs:
			key, node = heapq.heappop(open)
			coordinates = node.state
			if key < self._fieldDStarGetKey(startNode):
				break
				
			if node.cost > node.rhs:
				node.cost = node.rhs
				nodeDict[coordinates] = node
				for neighbor in self._fieldDStarGetNeighbors(node):
					self._FieldDStarUpdateState(neighbor)
			else:
				node.cost = float('inf')
				for neighbor in self._fieldDStarGetNeighbors(node) + [node]:
					self._FieldDStarUpdateState(neighbor)
		
	def _fieldDStarExtractPath(self, nodeList, startCoordinates, endCoordinates, optimize_vector, numTestPoints = 10):
		coordinates = startCoordinates
		path = [startCoordinates]
		
		def interpolatedConsecutiveCoordinates(p):
		# This function represents the 6 possible pairs of consecutive points
		# around an interpolated point
			if (p[0]%1 != 0):
				a = int(p[0])
				b = p[1]
				return [((a, b), (a, b+1)),
						((a, b+1), (a+1, b+1)),
						((a+1, b+1), (a+1, b)),
						((a+1, b), (a+1, b-1)),
						((a+1, b-1), (a, b-1)),
						((a, b-1), (a, b))]
			else:
				a = p[0]
				b = int(p[1])
				return [((a, b) ,(a+1, b)),
						((a+1, b), (a+1, b+1)),
						((a+1, b+1), (a, b+1)),
						((a, b+1), (a-1, b+1)),
						((a-1, b+1), (a-1, b)),
						((a-1, b), (a, b))]
						
		# node will always refer to the current point in the path
		while coordinates != endCoordinates:
			if (coordinates[0] % 1 != 0) or (coordinates[1] % 1 != 0): #interpolated point
				minCost = float('inf')
				nextPoint = None
				
				# we calculate the height of the current point based on linear interpolation
				if coordinates[0] % 1 != 0:
					h1 = self.map.getElevation((int(coordinates[0]), coordinates[1]))
					h2 = self.map.getElevation((int(coordinates[1])+1, coordinates[1]))
					
					height = h1*(1-(coordinates[0]%1)) + h2*(coordinates[0]%1)
				else:
					h1 = self.map.getElevation((coordinates[0], int(coordinates[1])))
					h2 = self.map.getElevation((coordinates[0], int(coordinates[1])+1))
				
					height = h1*(1-(coordinates[1]%1)) + h2*(coordinates[0]%1)
				
				# There should be six pairs; we put the best option into nextPoint with the associated cost into minCost
				for pair in interpolatedConsecutiveCoordinates(coordinates):
					# making sure that both coordinates in the pair are contained
					# inside the nodeList
					if (pair[0] in nodeList) and (pair[1] in nodeList):
						s_1 = nodeList[pair[0]]
						s_2 = nodeList[pair[1]]
					else:
						continue
					
					h_1 = self.map.getElevation(s_1.state)
					h_2 = self.map.getElevation(s_2.state)
					c_1 = s_1.cost
					c_2 = s_2.cost
					
					def f(y):
						# This is the function to be minimized
						prevCost = (1-y)*c_1 + y*c_2
						p = ((1-y)*pair[0][0] + y*pair[1][0], (1-y)*pair[0][1] + y*pair[1][1])
						h = (1-y)*h_1 + y*h_2
						
						path_length = math.sqrt((p[0]-coordinates[0])**2 + (p[1]-coordinates[1])**2) * self.map.resolution
						slope = math.degrees(math.atan((height - h) / path_length))
						grav = self.map.getGravity
						
						distWeight = self.explorer.distance(path_length)*optimize_on[0]
						timeWeight = self.explorer.time(path_length, slope)*optimize_on[1]
						energyWeight = self.explorer.energyCost(path_length, slope, gravity)*optimize_on[2]
						
						return prevCost + distWeight + timeWeight + energyWeight
					
					step = 1.0/numTestPoints
					
					testPoints = [step/2 + step * i for i in range(numTestPoints)]
					fPoints = [f(tp) for tp in testPoints]
					startPoint = testPoints[np.argmin(fPoints)]

					min = minimize(f, startPoint, method = 'SLSQP', bounds = ((0, 1))).x
					minResult = f(min)
					
					if minResult < minCost:
						minCost = minResult
						nextPoint = ((1-min)*pair[0][0] + min*pair[1][0], (1-min)*pair[0][1] + min*pair[1][1])
				
				if nextPoint:
					# if nextPoint exists, add it
					path.append(nextPoint)
					coordinates = nextPoint
				else:
					break
				
			else: # both coordinates are integers
				consecutiveNeighbors = self._fieldDStarGetConsecutiveNeighbors(coordinates)
				minCost = float('inf')
				nextPoint = None
				for pair in consecutiveNeighbors:
					print nodeList
					point, cost = self._fieldDStarComputeCost(nodeList[coordinates], nodeList[pair[0]], nodeList[pair[1]], optimize_vector)
					if cost < minCost:
						minCost = cost
						nextPoint = point
				path.append(point)
				coordinates = point
		
		return path
	
	def fieldDStarSearch(self, startCoords, endCoords, optimize_on, numTestPoints = 10):
		
		startNode = FieldDStarNode(startCoords, float('inf'), float('inf'))
		endNode = FieldDStarNode(endCoords, float('inf'), 0)
		
		open = []
		nodeDict = {} # This dictionary maps coordinates to FieldDStarNode objects.
					  # Only contains nodes that have been travelled to/are relevant
		nodeDict[startCoords] = startNode
		nodeDict[endCoords] = endNode
		
		heapq.heappush(open, (self._fieldDStarGetKey(endNode, endNode, optimize_on), endNode))
		self._fieldDStarComputeShortestPath(startNode, open)
		
		return self._fieldDStarExtractPath(nodeDict, startCoords, endNode, optimize_on, numTestPoints)
	
	def _toJSON(self, path, optimize_on, waypoints):
		'''
		This returns the list following "sequence." It doesn't really make sense
		for SEXTANT to actually do parts of the JSON file such as "creator."
		'''		
		sequence = []
		def empty():
			return {
						"geometry": {
							"type": "LineString",
							"coordinates": []
									},
						"derivedInfo": {
							"distanceList": [],
							"energyList": [],
							"timeList": [],
							"totalDistance": 0,
							"totalTime": 0,
							"totalEnergy": 0
										 },
						"type": "Segment"
					}
		lineString = {}
		
		def nextStation(lineString, firstStation = False): #firstStation denotes if the station is the first of the path
			AP = waypoints.pop(0)
			element = {}
			
			latLong = self.map.convertToLatLong(AP.coordinates)
			
			element["geometry"] = {"coordinates": latLong.latLongList(), "type": "Point"}
			element["type"] = "Station"
			
			# When we hit a station, we need to add the previous lineString to the sequence,
			# compute the total Dist, Time, and Energy
			if not firstStation:
				lineString["derivedInfo"]["totalDistance"] = sum(lineString["derivedInfo"]["distanceList"])
				lineString["derivedInfo"]["totalTime"] = sum(lineString["derivedInfo"]["timeList"])
				lineString["derivedInfo"]["totalEnergy"] = sum(lineString["derivedInfo"]["energyList"])
				
				sequence.append(lineString)
				
			return element
		
		for i, point in enumerate(path):
			# first set of if/elif statements creates the "Station" or "pathPoint" things,
			# second set of if/elif statements creates the "segments" connecting them
			if i == 0: # first element is a station
				sequence.append(nextStation(lineString, True))
				lineString = empty()
				lineString["geometry"]["coordinates"].append(self.map.convertToLatLong(point).latLongList())
			elif point == path[i-1]: # point is identical to the previous one and is thus a station
				sequence.append(nextStation(lineString))
				lineString = empty()
				lineString["geometry"]["coordinates"].append(self.map.convertToLatLong(point).latLongList())
			else: # This point is not a station and is thus a point inside the path
				latLongCoords = self.map.convertToLatLong(point).latLongList()
				
				lineString["geometry"]["coordinates"].append(latLongCoords)
				
				startState, endState = path[i-1], path[i]
				distance = self._aStarCostFunction(startState, endState, [1, 0, 0]) # distance
				energy = self._aStarCostFunction(startState, endState, [0, 0, 1]) # energy
				time = self._aStarCostFunction(startState, endState, [0, 1, 0]) # time
				
				lineString["derivedInfo"]["distanceList"].append(distance)
				lineString["derivedInfo"]["energyList"].append(energy)
				lineString["derivedInfo"]["timeList"].append(time)
		
		# Call nextStation at the end to make the final point a station
		sequence.append(nextStation(lineString))
		
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
	
class aStarSearchNode:
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

class FieldDStarNode:
	'''
	This class is intended to assist for the DStar search mode.
	This takes longer but is much more flexible, as it allows
	for more than the 8 cardinal directions.
	'''
	def __init__(self,coordinates,cost = float('inf'),rhs = float('inf')):
		self.coordinates = coordinates
		self.cost = cost
		self.rhs = rhs # default set to positive infinity