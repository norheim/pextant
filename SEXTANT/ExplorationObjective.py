import numpy as np
import math

class ActivityPoint:
	def __init__(self, coordinates, duration = 0, information = {}):
		self.coordinates = coordinates # coordinates is a tuple (row, column)
		self.duration = duration # I think we will make duration in minutes
		self.information = information # Various information about the activity point such as
									   # id and depth (for water missions)
		
	def setCoordinates(self, coordinates):
		self.coordinates = coordinates
	
	def setDuration(self, duration):
		self.duration = duration