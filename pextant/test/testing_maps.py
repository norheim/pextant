import numpy as np

'''
This document contains some plots to test on

All of them are 100x100 which should be a reasonable testing size
'''


def get_map(type):
	#Graph 1: Completely Flat
	if type == "flat":
		return np.zeros((100, 100))
	
	elif type == "maze":
	#Graph 2: Maze like obstacles
		maze_map = np.zeros((100, 100))

		for row in range(100):
			for col in range(100):
				if (0 <= row <= 80 and 20 <= col <= 40) or (20 <= row <= 100 and 60 <= col <= 80):
					maze_map[row][col] = 1000
		
		return maze_map
		
	elif type == "gradient":
		#Same type of shape as "maze", but has a gradient slope
		gradient_map = np.zeros((100, 100))

		for row in range(100):
			for col in range(100):
				if (0 <= row <= 80 and 20 <= col <= 40):
					gradient_map[row][col] = row
				elif (20 <= row <= 100 and 60 <= col <= 80):
					gradient_map[row][col] = 100 - row
		
		return gradient_map
	
	else:
		raise NameError("map type", type, "not found")
