import pyproj

def _longToUTMZone(longitude):
	# returns the UTM zone based on the longitude. Doesn't take into account the special zones.
	return (int((longitude + 180)/6.0)%60)+1

def _getZoneLetter(coordinate):
	# Figures out the zone letter of a LatLongCoord
	lat = coordinate.latitude
	
	UTMzdlChars="CDEFGHJKLMNPQRSTUVWXX"
	
	if -80<=lat and lat<=84:
		return UTMzdlChars[int((lat+80)/8)]
	else:
		# Not normally reached 
		print "No zdl: UTM is not valid for Lat " + str(lat)
		return None
	
def UTMToLatLong(utm_coordinate):
	'''
	Converts from WGS84 lat/long to NAD83 UTM
	'''
	x = utm_coordinate.easting
	y = utm_coordinate.northing
	zone = utm_coordinate.zone
	
	p1 = pyproj.Proj("+proj=utm +zone="+str(zone)+", +north +datum=WGS84")
	p2 = pyproj.Proj(proj='latlong', zone=zone, datum='WGS84')
	
	lon, lat = pyproj.transform(p1, p2, x, y)
	return (lat, lon)
	
	
def latLongToUTM(lat_long_coordinate):
	'''
	Coverts from NAD83 UTM to WGS84 lat/long
	'''
	x = lat_long_coordinate.longitude
	y = lat_long_coordinate.latitude
	zone=_longToUTMZone(x)
	zoneLetter = _getZoneLetter(lat_long_coordinate)
	
	p1 = pyproj.Proj(proj='latlong', datum='WGS84')
	p2 = pyproj.Proj(proj='utm', zone = zone, datum='WGS84')
	
	easting, northing = pyproj.transform(p1, p2, x, y)
	return (easting, northing, zone, zoneLetter)
	
	