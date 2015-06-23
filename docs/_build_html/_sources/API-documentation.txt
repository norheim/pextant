=================
API Documentation
=================

:class:`EnvironmentalModel` Objects
-------------------------------------

.. py:class:: EnvironmentalModel(elevation_map, resolution, maximum_slope, [planet="Earth", [NW_UTM]])

	Initialize an EnvironmentalModel object with an elevation map and coordinates

	:param elevation_map: A two-dimensional numpy array representing the elevation at each point.
	:param float resolution: The resolution of our map
	:param float maximum_slope: The maximum traversable slope
	:param string planet: Can be set to "Earth", "Mars", or "Moon". Will only be set to "Earth" for BASALT
	:param NW_UTM: A :py:class:`UTMCoord` object representing the North-western most corner of the map
	
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
	
:class:`ExplorerModel` Objects
-------------------------------------

.. py:class:: ExplorerModel(mass, gravity, [parameters = None])

	Initialize an object representing an explorer. Note that energy and time cost functions are missing

	:param float mass: The mass of the explorer
	:param float gravity: The gravity of the planet
	:param parameters: A parameters object. Currently serves no function
	
.. py:class:: Rover(mass, gravity, [parameters = None, constant_speed = 15, additional_energy = 1500])

	An instance of ExplorerModel representing a Rover. Contains all instance variables of ExplorerModel as well as:
	
	:param float speed: The constant speed that the rover moves at
	:param float P_e: The collection of all additional electronic components on the rover, estimated to be 1500W
	:param type: Set to 'Rover'
	
.. py:class:: Astronaut(mass, gravity, [parameters = None, constant_speed = 15, additional_energy = 1500])

	An instance of ExplorerModel representing a lunar Astronaut. Contains all instance variables of ExplorerModel as well as:
	
	:param float speed: The constant speed that the rover moves at
	:param float P_e: The collection of all additional electronic components on the rover, estimated to be 1500W
	:param type: Set to 'Rover'
