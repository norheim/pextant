=================
Basic API Documentation
=================

This is probably all that will be needed to be accessed from "outside"

Coordinate objects
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

Elevation Map Objects and Methods
------------------------------
	
.. py:method:: loadElevationMap(file, maxSlope = 15, planet = "Earth", NWCorner = None, SECorner = None, desiredRes = None, 
					no_val = -10000)
					
	Returns a EnvironmentalModel object from a (preferably geoTIFF) file. Parameters from NWCorner and onwards are for downscaling
	or cropping the geoTIFF file.
	
:class:`ActivityPoint`
-------------------------------------

The ActivityPoint object represents points of interest for the explorer, likely spots
for observation or data collection. It's possible that future versions of Pextant may
have extensions of ActivityPoint.

.. py:class:: ActivityPoint(coordinates, duration = 0, uuid = None)

	Initialize an ActivityPoint representing a waypoint.

	:param coordinates: A tuple representing the location of the waypoint
	:param float duration: The amount of time spent at the ActivityPoint, in seconds.
	:param string uuid: A uuid value for the activityPoint

.. py:method:: setCoordinates(coordinates)

	Sets the coordinates of the ActivityPoint to a new value. This can be a row/column tuple, a UTMCoord, or a LatLongCoord Object.
	
.. py:method:: setDuration(duration)
	
	Sets the duration of the activityPoint.

:class:`PathFinder`
------------------------------------

.. py:class:: PathFinder(explorer_model, environmental_model)

	Initialize a PathFinder Object used to calculate and analyse paths.
	
	:param explorer_model: An ExplorerModel object representing the explorer
	:param environmental_model: An EnvironmentalModel object representing the map
	
.. py:method:: aStarSearch(start, end, optimize_on)
	
	Returns a path through the start node and the end node using the A* search algorithm.
	
	:param start: An ActivityPoint object, and the starting point of the search.
	:param end: Also an ActivityPoint object
	:param optimize_on: A string denoting what factor to optimize on, such as "Energy" or "Time"
	
.. py:method:: fieldDStarSearch(start, end, optimize_on, numTestPoints = 11)

	Returns a path through the start node and the end node using the Field D* algorithm.
	Longer processing time than A*, but allows for more than the 8 cardinal directions, resulting in
	more "fluid" paths.

	:param start: An ActivityPoint object, and the starting point of the search.
	:param end: Also an ActivityPoint object
	:param optimize_on: A string denoting what factor to optimize on, such as "Energy" or "Time"
	:param int numTestPoints: A number used in the costFunction calculations. Higher values will involve more accuracy but increased time.
	
.. py:method:: aStarCompletePath(optimize_on, activityPoints, returnType = "JSON", fileName = None)
	
	Returns a path through all of the ActivityPoint objects in exploration_objectives in order. The path takes the form
	of a long list of row/column tuples. Currently runs with the A* search algorithm.
	
	:param optimize_on: Determine what factor to optimize on (can be "Energy", "Time", or "Distance")
	:param activityPoitns: A list of activityPoint objects representing the places to visit, in order
	:param returnType: A string representing the format of the path to be returned. Options are 'tuple', 'JSON', and 'csv'
	:param fileName: The optional name of the file to be written to
	
.. py:method:: fieldDStarCompletePath(optimize_on, waypoints, returnType = "JSON", fileName = None, numtestPoints = 11)

	Similar to aStarCompletePath, except uses the field D* algorithm. Currently still under development.
