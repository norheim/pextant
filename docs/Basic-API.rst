=================
Basic API Documentation
=================

This is probably all that will be needed to be accessed from "outside"

.. py:method:: loadElevationMap(file, maxSlope = 15, planet = "Earth", NWCorner = None, SECorner = None, desiredRes = None, 
					no_val = -10000)
					
	Returns a EnvironmentalModel object from a (preferably geoTIFF) file. Parameters from NWCorner and onwards are for downscaling
	or cropping the geoTIFF file.
	
