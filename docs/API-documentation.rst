=================
API Documentation
=================

:class:`EnvironmentalModel` Objects
-------------------------------------

An EnvironmentalModel essentially contains an elevation map and information about which 

.. py:class:: EnvironmentalModel(elevation_map, resolution, maximum_slope, [planet="Earth", [NW_UTM]])

	Initialize an EnvironmentalModel object with an elevation map and obstacle information.

	:param elevation_map: A 2D numpy array representing the elevation at each point.
	:param float resolution: The resolution of our map
	:param float maximum_slope: The maximum traversable slope
	:param string planet: Can be set to "Earth", "Mars", or "Moon". Will only be set to "Earth" for BASALT, currently serves no use.
	:param NW_UTM: A :py:class:`UTMCoord` object representing the North-western most corner of the map

	Instance Variables:
	
	* elevations: a 2D numpy array representing the elevation at each point.
	* slopes: a 2D numpy array representing the slope at each point.
	* obstacles: a 2D numpy array representing whether or not each point exceeds the value for the maximum_slope.
	* resolution: a float representing, in meters, the resolution of the elevation map.
	* numRows, numCols: number of rows and columns in the elevation map. Included for convenience.
	* planet: the planet (probably "Earth").
	* NW_UTM: the Northwestern-most point of the map as a UTMCoord object.
	* special_obstacles: these are obstacles from the setObstacle method.
	
.. py:method:: setMaxSlope(slope)

	Sets the maximum traversable slope. All points with a higher slope are marked as impassable obstacles.

	:param float slope: The maximum allowed slope
	
.. py:method:: setObstacle(coordinates)

	Sets an obstacle at the location designated by the coordinates. Likely used for regions that are
	impassable due to reasons other than slope.
	
	:param coordinates: Can be a UTMCoord, LatLongCoord, or tuple.
	
.. py:method:: eraseObstacle(coordinates)

	Erases an obstacle at the location designated by the coordinates.
	
.. py:method:: getElevation(coordinates)

	Retrieve the elevation at the location designated by the coordinates.
	
.. py: method:: getSlope(coordinates)
	
	Retrieve the slope at the location designated by the coordinates.
	
.. py: method:: isPassable(coordinates)

	Returns a boolean representing if the location designated by the coordinates can be traversed.
	
.. py: method:: convertToRowCol(coordinates)

	Converts any type of coordinates (UTMCoord, LatLongCoord, tuple) into a row/column tuple.
	
.. py: method:: convertToUTM(coordinates)

	Converts any type of coordinates (UTMCoord, LatLongCoord, tuple) into an UTMCoord object

.. py: method:: convertToLatLong(coordinates)

	Converts any type of coordinates (UTMCoord, LatLongCoord, tuple) into a LatLongCoord object

.. py:method:: loadElevationMap(file, [maxSlope = 15, planet = "Earth"])

	Returns an EnvironmentalModel object given a geoTIFF or text file. Currently only reliably works with NAD83
	and files using UTM rather than lat/long. 

	:param file: A file location of a geoTIFF or a text file representing an elevation map. 
	
Coordinate-representing objects
--------------------------

Three types of objects are used to represent coordinates: tuples, which are assumed to be a row/column
pair, UTMCoord, and LatLongCoord. At the moment the assumption is that all Coordinate-representing objects
use NAD83, though this can be easily changed. Generally, any function requiring coordinates will accept
any of these three types of coordinate objects.

.. py:class:: UTMCoord(easting, northing, zone_number, zone_letter)

	Initializes an object representing UTM coordinates of a point

	:param float easting: Easting value of the coordinate
	:param float northing: Northing value of the coordinate
	:param int zone: The zone number of the coordinate
	:param zone_letter: The zone letter of the coordinate (future versions may accept "North" or "South" as well)
	
.. py:class:: LatLongCoord(lat, long)

	Initializes an object representing a point by latitude and longitude

	:param float latitude: Latitude value (values north of the equator are positive, values south are negative)
	:param float longitude: Longitude value (values east of the prime meridian are positive, values west are negative)
	
:class:`ExplorerModel`
-------------------------------------

All ExplorerModel-type objects contain basic information about a unit, as well as distance, time, and energy cost functions.
A few assumptions are made:

* The velocity and metabolic rate of an explorer is solely a function of slope
* The explorer will not become "tired" as time goes on

.. py:class:: ExplorerModel(mass, gravity, [parameters = None])

	Initialize an object representing an explorer. Note that energy and time cost functions are missing

	:param float mass: The mass of the explorer
	:param float gravity: The gravity of the planet
	:param parameters: A parameters object which can be used to calculate shadowing. As the current
		version of Pextant does not support shadowing this currently has no purpose.
	
.. py:method:: distance(path_length)
	
	Returns the distance given a path length.

.. py:method:: velocity(slope)

	Returns the velocity of the explorer given the slope of the surface.

.. py:method:: time(path_length, slope)

	Returns the amount of time it takes to cross a path given the path length and slope. Calculated by dividing distance by velocity.

.. py:method:: energyRate(slope)

	Returns the rate of energy expenditure based on the slope of the ground

.. py:method:: energy(path_length, slope)

	Returns the amount of energy it takes to cross a path given the path length and slope. Calculated by multiplying energyRate by time.
	
.. py:class:: Rover(mass, gravity, [parameters = None, constant_speed = 15, additional_energy = 1500])

	An instance of ExplorerModel representing a Rover. Contains all instance variables of ExplorerModel as well as:
	
	:param float speed: The constant speed that the rover moves at
	:param float P_e: The collection of all additional electronic components on the rover, estimated to be 1500W
	:param type: Set to 'Rover'

	Includes specialized metabolic cost functions from Carr 2001.

.. py:class:: Astronaut(mass, gravity, [parameters = None])

	An instance of ExplorerModel representing a lunar Astronaut. Contains all instance variables of ExplorerModel as well as:

	:param type: Set to 'Astronaut'

	Includes metabolic cost functions from Santee 2001, as well as a velocity function from Marquez 2007 (based on
	data from Waligoria and Horrigan 1975).
	
.. py:class:: BASALTExplorer(mass, gravity, [parameters = None])
	
	An instance of ExplorerModel representing a BASALT scientist. Currently empty; will be completed after an analysis
		of data from the August COTM missing, in order to derive a velocity function.
	
:class:`ActivityPoint`
-------------------------------------

The ActivityPoint object represents points of interest for the explorer, likely spots
for observation or data collection. It's possible that future versions of Pextant may
have extensions of ActivityPoint.

.. py:class:: ActivityPoint(coordinates, [duration = 0, information])

	Initialize an ActivityPoint representing a waypoint.

	:param coordinates: A tuple representing the location of the waypoint
	:param float duration: The amount of time spent at the ActivityPoint, in seconds.
	:param dict information: A dictionary containing information that would be written into a JSON file.

.. py:method:: setCoordinates(coordinates)

	Sets the coordinates of the ActivityPoint to a new value. This can be a row/column tuple, a UTMCoord, or a LatLongCoord Object.
	
.. py:method:: setDuration(duration)
	
	Sets the duration of the activityPoint.
	
:class:`PathFinder`
------------------------------------

.. py:class:: PathFinder(explorer_model, environmental_model, exploration_objectives)

	Initialize a PathFinder Object used to calculate and analyse paths.
	
	:param explorer_model: An ExplorerModel object representing the explorer
	:param environmental_model: An EnvironmentalModel object representing the map
	:param exploration_objectives: A list of ActivityPoint objects representing, in order, the waypoints
	
.. py:method:: completePath(optimize_on)
	
	Returns a path through all of the ActivityPoint objects in exploration_objectives in order. The path takes the form
	of a long list of row/column tuples.
	
	:param optimize_on: Determine what factor to optimize on (can be "Energy", "Time", or "Distance")
	
.. py:method:: toJSON(optimize_on)

	Same as above, but instead returns a list in a JSON-style format. I tried to mimic the style of the JSON
	files that were used in xGDS.
	
.. py:method:: analysePath(path, [factor = "Energy"])

	Provides a lot of information about a particular path, depending on what factor is set to.

	:param path: A path like the one returned by completePath
	:param factor: Determines what to analyse. Currently can be set to "Energy", "Time", "Distance",
					"CumulativeEnergy", "CumulativeTime", "CumulativeDistance"
					
