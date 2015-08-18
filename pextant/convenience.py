def getWeightedElevation(envModel, coordinates):
	# returns the interpolated height of coordinates that aren't integers
	x, y = coordinates
	if (x % 1 != 0) and (y % 1 == 0):
		weight = x%1
		h1 = envModel.getElevation((int(x), y))
		h2 = envModel.getElevation((int(x)+1, y))
		return h2*weight + h1*(1-weight)
	elif (x % 1 == 0) and (y % 1 != 0):
		weight = y%1
		h1 = envModel.getElevation((x, int(y)))
		h2 = envModel.getElevation((x, int(y)+1))
		return h1*weight + h2*(1-weight)
	elif (x % 1 == 0) and (y % 1 == 0):
		return envModel.getElevation((x, y))
	else:
		return None
		
def getWeightedCost(nodeDict, coordinates):
	x, y = coordinates
	if (x % 1 != 0) and (y % 1 == 0):
		weight = x%1
		c1 = nodeDict[(int(x), y)].cost
		c2 = nodeDict[(int(x)+1, y)].cost
		return c2*weight + c1*(1-weight)
	elif (x % 1 == 0) and (y % 1 != 0):
		weight = y%1
		c1 = nodeDict[(x, int(y))].cost
		c2 = nodeDict[(x, int(y)+1)].cost
		return c1*weight + c2*(1-weight)
	elif (x % 1 == 0) and (y % 1 == 0):
		return nodeDict[(x, y)].cost
	else:
		return None