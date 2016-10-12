import pyproj


class UTMCoord(object):
    """
    Represents a UTM coordinate. All UTM Coordinates should be expressed in this form.
	"""

    def __init__(self, easting, northing, zone, zone_letter='T'):
        self.easting = easting
        self.northing = northing
        self.zone = zone
        self.zoneLetter = zone_letter  # Note: 'T' is the zone letter of all but the northernmost point of Idaho
        # 'Q' is the zone letter of Hawaii

    def unpack(self):
        return self.easting, self.northing, self.zone, self.zoneLetter

    def __str__(self):
        return "UTM Coordinate " + str(self.easting) + "E," + str(self.northing) + "N, zone" + str(
            self.zone) + self.zoneLetter + '\n'


class LatLongCoord(object):
    '''
	Represents a latitude and longitude pair. All Lat/Long Coordinates should be expressed in this form.
	'''

    def __init__(self, lat, long):
        self.latitude = lat
        self.longitude = long

    def __str__(self):
        return "LatLong Coordinate " + str(self.latitude) + ',' + str(self.longitude) + '\n'

    def latLongList(self):
        return [self.latitude, self.longitude]

    def longLatList(self):
        return [self.longitude, self.latitude]


def _longToUTMZone(longitude):
    # returns the UTM zone based on the longitude. Doesn't take into account the special zones.
    return (int((longitude + 180) / 6.0) % 60) + 1


def _getZoneLetter(coordinate):
    # Figures out the zone letter of a LatLongCoord
    lat = coordinate.latitude

    UTMzdlChars = "CDEFGHJKLMNPQRSTUVWXX"

    if -80 <= lat and lat <= 84:
        return UTMzdlChars[int((lat + 80) / 8)]
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

    p1 = pyproj.Proj("+proj=utm +zone=" + str(zone) + ", +north +datum=WGS84")
    p2 = pyproj.Proj(proj='latlong', zone=zone, datum='WGS84')

    lon, lat = pyproj.transform(p1, p2, x, y)
    return (lat, lon)


def latLongToUTM(lat_long_coordinate):
    '''
	Coverts from NAD83 UTM to WGS84 lat/long
	'''
    x = lat_long_coordinate.longitude
    y = lat_long_coordinate.latitude
    zone = _longToUTMZone(x)
    zoneLetter = _getZoneLetter(lat_long_coordinate)

    p1 = pyproj.Proj(proj='latlong', datum='WGS84')
    p2 = pyproj.Proj(proj='utm', zone=zone, datum='WGS84')

    easting, northing = pyproj.transform(p1, p2, x, y)
    return UTMCoord(easting, northing, zone, zoneLetter)
