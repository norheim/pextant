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