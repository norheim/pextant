import numpy as np
import math
from geoshapely import *

class ActivityPoint:
	def __init__(self, coordinates, duration = 0, uuid = None):
		self.coordinates = coordinates # coordinates is either a row/column tuple, a LatLongCoord, or a UTMCoord
		self.duration = duration # For consistency duration should be in seconds
		self.UUID = uuid
		self.geopoint = GeoPoint(LAT_LONG, coordinates.latitude, coordinates.longitude)
		
	def setCoordinates(self, coordinates):
		self.coordinates = coordinates
	
	def setDuration(self, duration):
		self.duration = duration