===================================
HOW IT WORKS
===================================

The Components of Path Planning
-----------------------------------

A PEXTANT-planned path requires three distinct components:

* An EnvironmentalModel, which contains DEM information and obstacle locations
* An ExplorerModel, containing the cost functions of each explorer
* A list of waypoints, in the order in which they will be explored

Calculating Cost Functions
-----------------------------------

The Distance Cost Function

	The Distance Cost Function of a segment is simply the length of that segment
	
Calculating Velocity

	PEXTANT makes the assumption that the velocity of an explorer is based solely on
	the slope of the path traversed. 
	
	
The A Star Search Algorithm
-----------------------------------


The Field D Star Search Algorithm
-----------------------------------