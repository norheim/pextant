class ActivityPoint:
	def __init__(self, coordinates, duration = 0, uuid = None):
		self.coordinates = coordinates # coordinates is either a row/column tuple, a LatLongCoord, or a UTMCoord
		self.duration = duration # For consistency duration should be in seconds
		self.UUID = uuid
		
	def setCoordinates(self, coordinates):
		self.coordinates = coordinates
	
	def setDuration(self, duration):
		self.duration = duration