import numpy as np
from pandas.tslib import _NaT

from transform import *
from osgeo import gdal, osr

class EnvironmentalModel(object):
    '''
	This class ultimately represents an elevation map + all of the traversable spots on it.
	
	Public functions:
	
	setMaxSlope(slope) - sets the maximum slope that can be traversed
	setObstacle(coordinates), eraseObstacle(coordinates) - set or erase an obstacle at certain coordinates
	getElevation(coordinates), getSlope(coordinates) - get Elevation or Slope at certain coordinates
	isPassable(coordinates) - determines if a certain coordinate can be traversed
	convertToRowCol(coordinates), convertToUTM(coordinates), convertToLatLong(coordinates) - converts from one coordinate system to another
	loadElevationMap(fileName) - load an elevation map from a geoTIFF or text file
	'''

    def __init__(self, elevation_map, resolution, maxSlope, NW_Coord, planet="Earth", uuid=None):
        self.elevations = elevation_map  # this is a numpy 2D array
        self.resolution = resolution  # float expected in meters/pixel
        [gx, gy] = np.gradient(elevation_map, resolution, resolution)
        #self.gx = gx
        #self.gy = gy
        # self.slopes = np.degrees(np.arctan(np.sqrt(np.add(np.square(gx),np.square(gy))))) # I think this syntax is correct
        # we want atan(sqrt(gx^2+gy^2)) in degrees
        self.slopes = np.degrees(
            np.arctan(np.sqrt(np.add(np.square(gx), np.square(gy)))))  # Combining for now for less RAM usage
        self.numRows, self.numCols = elevation_map.shape
        self.obstacles = self.slopes <= maxSlope  # obstacles is basically an "isPassable" function
        self.planet = planet
        self.NW_UTM = self.convertToUTM(NW_Coord)  # a UTMCoord object, default set to Boston
        self.special_obstacles = set()  # a list of coordinates of obstacles are not identified by the slope
        self.UUID = uuid

    def getGravity(self):
        if self.planet == 'Earth':
            return 9.81
        elif self.planet == 'Moon':
            return 1.622

    def setMaxSlope(self, maxSlope):
        self.obstacles = self.slopes <= maxSlope
        for obstacle in self.special_obstacles:
            self.setObstacle(obstacle)  # "special obstacles" still should not be traversable

    def setObstacle(self, coordinates):
        row, col = self.convertToRowCol(coordinates)  # of the form (row, column)
        self.obstacles[row][col] = False
        if coordinates not in self.special_obstacles:
            self.special_obstacles.add(coordinates)

    def eraseObstacle(self, coordinates):
        row, col = self.convertToRowCol(coordinates)
        self.obstacles[row][col] = True
        if coordinates in self.special_obstacles:
            self.special_obstacles.remove(coordinates)

    def getElevation(self, coordinates):
        row, col = self.convertToRowCol(coordinates)
        return self.elevations[row][col]

    def getSlope(self, coordinates):
        row, col = self.convertToRowCol(coordinates)
        return self.slopes[row][col]

    def setNoVal(self, noValResult):
        for i, row in enumerate(self.elevations):
            for j, item in enumerate(row):
                if item == noValResult:
                    self.elevations[i][j] = None
                    self.slopes[i][j] = None
                    self.obstacles[i][j] = True

    def _inBounds(self, coordinates):
        # determines if a state is within the boundaries of the environmental model
        # a state is a tuple of the form (row, column)
        row, col = coordinates
        return (0 <= row < self.numRows) and (0 <= col < self.numCols)

    def isPassable(self, coordinates):
        # determines if coordinates can be passed through
        row, col = self.convertToRowCol(coordinates)  # coordinates is a tuple
        if self._inBounds(coordinates):
            return self.obstacles[row][col]
        else:
            return False

    def _UTMtoRowCol(self, UTM):
        '''
		Converts from UTMCoord object to a tuple representing the row and the column
		in the matrix. There may be minor rounding issues
		
		As of right now both UTMtoRowCol and RowColToUTM only work if we do not cross a UTM line.
		Eventually for completeness sake this should probably be changed
		'''
        if str(UTM.zone) == str(self.NW_UTM.zone):
            row = round((self.NW_UTM.northing - UTM.northing) / self.resolution)
            # for column due to image reversal need to do non obvious thing:
            col = round((UTM.easting - self.NW_UTM.easting) / self.resolution)
            return row, col
        else:
            print "WARNING: map is in UTM zone " + repr(self.NW_UTM.zone)
            print "Zone given: " + repr(UTM.zone)
            pass  # Later I could figure out how to account for zone changes just in case a site is
            # just on the border of a UTM zone.

    def _rowColToUTM(self, coordinates):
        '''
		The reverse of the above function.
		'''
        row, col = coordinates

        xNW, yNW = self.NW_UTM.easting, self.NW_UTM.northing
        x = xNW + col * self.resolution
        y = yNW - row * self.resolution
        return UTMCoord(x, y, self.NW_UTM.zone, self.NW_UTM.zoneLetter)
    #TODO make static function
    def convertToRowCol(self, position):
        """
		This should be called before every function in EnvironmentalModel. Converts UTMCoord or LatLongCoord
		objects into a tuple representing the row and the column of the environmental map closest to that location.
		"""
        conversion = None
        if isinstance(position, UTMCoord):
            conversion = self._UTMtoRowCol(position)
        elif isinstance(position, LatLongCoord):
            conversion = self._UTMtoRowCol(latLongToUTM(position))
        elif isinstance(position, tuple):
            conversion= position
        else:
            print "ERROR: only accepts UTMCoord, LatLongCoord, and tuple objects"
            print "Received " + repr(type(position)) + " object"
            return 0

        return tuple([int(i) for i in conversion])

    def convertToLatLong(self, position):
        '''
		Converts a row/column or UTM coordinate into a LatLongCoord
		'''
        if type(position) is UTMCoord:
            lat, lon = UTMToLatLong(position)
            return LatLongCoord(lat, lon)
        elif type(position) is LatLongCoord:
            return position
        elif type(position) is tuple:
            UTM = self._rowColToUTM(position)
            lat, lon = UTMToLatLong(UTM)
            return LatLongCoord(lat, lon)
        else:
            print "ERROR: only accepts UTMCoord, LatLongCoord, and tuple objects"
            print "Received " + repr(type(position)) + " object"
            return 0

    def convertToUTM(self, position):
        '''
		Converts a row/column coordinate or LatLongCoord into a UTMCoord.
		'''
        if type(position) is UTMCoord:
            return position
        elif type(position) is LatLongCoord:
            easting, northing, zoneNumber, zoneLetter = latLongToUTM(position)
            return UTMCoord(easting, northing, zoneNumber, zoneLetter)
        elif type(position) is tuple:
            return self._rowColToUTM(position)
        else:
            print "ERROR: only accepts UTMCoord, LatLongCoord, and tuple objects"
            print "Received " + repr(type(position)) + " object"
            return 0


def loadElevationMap(filePath, maxSlope=15, planet='Earth', nw_corner=None, se_corner=None, desired_res=None,
                     no_val=-10000, zone=None, zone_letter=None):
    '''
	Creates a EnvironmentalModel object from either a geoTiff file or a text file.
	
	Issue: computer sometimes freezes whenever loading a very large geoTIFF file (1GB+)

	Current modification: trying to enable loading a square sector of a geoTIFF file.
	2 new optional inputs: NWCorner and SE corner - UTM or LatLong coordinates of the subregion
	we would like to analyze.
	
	Using the parameter desiredRes doesn't work very well if there are a significant number of
	"unknown" points (usually denoted by a placeholder like -10000).
	'''
    # filePath is a string representing the location of the file
    extension = filePath.split('.')[-1]  # this should be the file extension

    if extension == 'txt':  # This likely needs some updating though it probably won't be used anytime soon
        f = open(filePath, r)
        inputs = []  # the values that end up in inputs will become numCols, numRows, xllcorner, yllcorner, cellsize, and NODATA_value
        for i in range(6):
            inputs.append(f.readline().split(' ')[-1])  # this should work given the current format
        numCols = inputs[0]
        numRows = inputs[1]
        map_array = np.empty([numRows, numCols])  # initializes an empty array
        for i in range(numRows):
            x = f.readline().split(' ')
            if len(x) == numCols:
                map_array[i] = x
            else:
                print "ERROR: expected " + str(numCols) + " columns. Got " + str(len(x)) + " columns"
                return 0
        return EnvironmentalModel(map_array, inputs[4], maxSlope, planet)
    elif extension == 'tif':
        # NOTE: Currently, SEXTANT only supports geoTIFF files that use the UTM projection and have "north up"
        gdal.UseExceptions()

        # copied from stackexchange
        dataset = gdal.Open(filePath)
        band = dataset.GetRasterBand(1)
        proj = dataset.GetProjection()

        srs = osr.SpatialReference(wkt=proj)
        projcs = srs.GetAttrValue('projcs')  # This will be a string that looks something like
        # "NAD83 / UTM zone 5N"...hopefully

        if projcs and (zone is None and zone_letter is None):  # projcs is not None for the government Hawaii data
            zone = int(projcs.split(' ')[-1][0:-1])
            zone_letter = projcs.split(' ')[-1][-1]

        dataset_info = dataset.GetGeoTransform()
        # returns a list of length 6. Indices 0 and 3 are the easting and northing values of the upper left corner.
        # Indices 1 and 5 are the w-e and n-s pixel resolutions, index 5 is always negative. Indicies 2 and 4 are
        # set to zero for all maps pointing in a "North up" type projection; for now we will only be using maps where
        # North is set to up.
        nw_easting = dataset_info[0]
        nw_northing = dataset_info[3]

        resolution = dataset_info[1]
        # SEXTANT does not support maps where the x-resolution differs from the y-resolution at the moment
        if abs(resolution) != abs(dataset_info[5]):
            raise ValueError('Resolution in x,y = ' + str(resolution) + "," + str(dataset_info[5]) + " do not match")

        buf_x = None
        buf_y = None
        if desired_res:
            buf_x = round(dataset.RasterXSize * dataset_info[1] / desired_res)
            buf_y = round(dataset.RasterYSize * dataset_info[1] / desired_res)
        else:
            desired_res = resolution # will need desiredRes later on

        if nw_corner is None and se_corner is None:  # No NW and SE corner implies we want the entire map
            # converts from a raster band to a numpy array
            map_array = band.ReadAsArray(buf_xsize=buf_x, buf_ysize=buf_y)
        else:
            nw_corner = nw_corner if isinstance(nw_corner, UTMCoord) else latLongToUTM(nw_corner)
            se_corner = se_corner if isinstance(se_corner, UTMCoord) else latLongToUTM(se_corner)

            top, bot = nw_corner.northing, se_corner.northing
            left, right = nw_corner.easting, se_corner.easting

            if bot > top or left > right:
                print "ERROR with NWCorner and SECorner"
                print "NWCorner: " + str(nw_corner) + "SWCorner: " + str(se_corner)
                return 0

            left = max(left, nw_easting)
            ne_easting = nw_easting + dataset_info[1] * (dataset.RasterXSize - 1)
            right = min(right, ne_easting)

            top = min(top, nw_northing)
            se_northing = nw_northing + dataset_info[5] * (dataset.RasterYSize - 1)
            bot = max(bot, se_northing)

            x_offset = int((left - nw_easting) / resolution)
            x_size = int((right - nw_easting) / resolution) + 1 - x_offset
            y_offset = int((nw_northing - top) / resolution)
            y_size = int((nw_northing - bot) / resolution) + 1 - y_offset

            map_array = band.ReadAsArray(x_offset, y_offset, x_size, y_size, buf_x, buf_y).astype(np.float)

            nw_easting = nw_corner.easting
            nw_northing = nw_corner.northing

        nw_coord = UTMCoord(nw_easting, nw_northing, zone, zone_letter)

        return EnvironmentalModel(map_array, desired_res, maxSlope, nw_coord, planet)
    else:
        raise ValueError("ERROR: expected txt or tif file. Received " + extension + " type file")

import re
def loadElevationsLite(file_path):
    gdal.UseExceptions()
    dataset = gdal.Open(file_path)

    all_info = {
        "width": dataset.RasterXSize,
        "height": dataset.RasterYSize
    }

    dataset_info = dataset.GetGeoTransform()
    nw_easting = dataset_info[0]
    nw_northing = dataset_info[3]
    all_info.update({
        "nw_easting" : nw_easting,
        "nw_northing" : nw_northing,
        "resolution" : dataset_info[1]
    })

    proj = dataset.GetProjection()
    srs = osr.SpatialReference(wkt=proj)
    projcs = srs.GetAttrValue('projcs')   # "NAD83 / UTM zone 5N"...hopefully
    regex_result = re.search('zone\s(\d+)(\w)', projcs)
    zone_number = regex_result.group(1)
    zone_letter = regex_result.group(2)

    all_info.update({
        "zone" : zone_number,
        "zone_letter" : zone_letter,
        "nw_geo_point" : GeoPoint(UTM(zone_number), nw_easting, nw_northing)
    })

    return dataset, all_info

from geoshapely import *
def loadElevationMapExp(file_path, maxSlope=15, planet='Earth', nw_corner=None, se_corner=None, desired_res=None,
                     no_val=-10000):
    '''
	Creates a EnvironmentalModel object from either a geoTiff file or a text file.

	Issue: computer sometimes freezes whenever loading a very large geoTIFF file (1GB+)

	Current modification: trying to enable loading a square sector of a geoTIFF file.
	2 new optional inputs: NWCorner and SE corner - GeoPoints

	Using the parameter desiredRes doesn't work very well if there are a significant number of
	"unknown" points (usually denoted by a placeholder like -10000).
	'''
    dataset, info = loadElevationsLite(file_path)
    width, height, resolution = info["width"], info["height"], info["resolution"]

    map_nw_corner = info["nw_geo_point"]
    XY = Cartesian(map_nw_corner, resolution)
    map_se_corner = GeoPoint(XY, width, height)

    map_box = LineString([(p.x, p.y) for p in [map_nw_corner, map_se_corner]]).envelope
    selection_box = LineString([(p.x, p.y) for p in [nw_corner, se_corner]]).envelope
    intersection_box = map_box.intersection(selection_box)

    inter_easting, inter_northing = np.array(intersection_box.bounds).reshape((2,2)).transpose()
    intersection_box_geo = GeoPolygon(UTM(info["zone"]), inter_easting, inter_northing)
    inter_x, inter_y = intersection_box_geo.to(XY)

    x_offset, max_x = inter_x
    max_y, y_offset = inter_y
    x_size = max_x - x_offset + 1
    y_size = max_y - y_offset + 1

    buf_x = None
    buf_y = None
    if desired_res:
        buf_x = int(x_size * resolution / desired_res)
        buf_y = int(y_size * resolution / desired_res)
    else:
        desired_res = resolution

    band = dataset.GetRasterBand(1)
    map_array = band.ReadAsArray(x_offset, y_offset, x_size, y_size, buf_x, buf_y).astype(np.float)
    nw_coord = UTMCoord(inter_easting.min(), inter_northing.max(), info["zone"], info["zone_letter"])

    return EnvironmentalModel(map_array, desired_res, maxSlope, nw_coord, planet)

