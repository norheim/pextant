from geoshapely import *
from osgeo import gdal, osr
import re

class EnvironmentalModel(object):
    '''
    This class ultimately represents an elevation map + all of the traversable spots on it.

    Public functions:
	
	setMaxSlope(slope) - sets the maximum slope that can be traversed
	setObstacle(coordinates), eraseObstacle(coordinates) - set or erase an obstacle at certain coordinates
	getElevation(coordinates), getSlope(coordinates) - get Elevation or Slope at certain coordinates
	isPassable(coordinates) - determines if a certain coordinate can be traversed
	convertToRowCol(coordinates), convertToUTM(coordinates), convertToLatLong(coordinates) - converts from one coordinate system to another
	'''
    def __init__(self, elevation_map, resolution, maxSlope, NW_Coord, planet="Earth", uuid=None):
        self.elevations = elevation_map  # this is a numpy 2D array
        self.resolution = resolution  # float expected in meters/pixel
        [gx, gy] = np.gradient(elevation_map, resolution, resolution)
        self.slopes = np.degrees(
            np.arctan(np.sqrt(np.add(np.square(gx), np.square(gy)))))  # Combining for now for less RAM usage
        self.numRows, self.numCols = elevation_map.shape
        self.obstacles = self.slopes <= maxSlope  # obstacles is basically an "isPassable" function
        self.planet = planet
        self.ROW_COL = Cartesian(NW_Coord, resolution, reverse=True)
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
        row, col = coordinates
        if self._inBounds(coordinates):
            return self.obstacles[row][col]
        else:
            return False

    def convertToRowCol(self, coordinates):
        if isinstance(coordinates, tuple):
            return coordinates
        else:
            return coordinates.to(self.ROW_COL)

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

def selectMapSection(dataset, info, geo_envelope=None, desired_res=None):
    """

    :param dataset:
    :param info:
    :param geo_envelope:
    :type geo_envelope pextant.geoshapely.GeoEnvelope
    :param desired_res:
    :return:
    """
    nw_corner, se_corner = geo_envelope.getBounds()
    width, height, resolution = info["width"], info["height"], info["resolution"]

    map_nw_corner = info["nw_geo_point"]
    XY = Cartesian(map_nw_corner, resolution)
    map_se_corner = GeoPoint(XY, width, height)

    selection_nw_corner = map_nw_corner if nw_corner is None else nw_corner
    selection_se_corner = map_se_corner if se_corner is None else se_corner

    map_box = GeoPolygon([map_nw_corner, map_se_corner]).envelope
    selection_box = GeoEnvelope(selection_nw_corner, selection_se_corner).envelope
    intersection_box = map_box.intersection(selection_box)

    inter_easting, inter_northing = np.array(intersection_box.bounds).reshape((2, 2)).transpose()
    intersection_box_geo = GeoPolygon(UTM(info["zone"]), inter_easting, inter_northing)
    inter_x, inter_y = intersection_box_geo.to(XY)

    x_offset, max_x = inter_x
    max_y, y_offset = inter_y
    x_size = max_x - x_offset
    y_size = max_y - y_offset

    buf_x = None
    buf_y = None
    if desired_res:
        buf_x = int(x_size * resolution / desired_res)
        buf_y = int(y_size * resolution / desired_res)
    else:
        desired_res = resolution

    band = dataset.GetRasterBand(1)
    map_array = band.ReadAsArray(x_offset, y_offset, x_size, y_size, buf_x, buf_y).astype(np.float)
    nw_coord = GeoPoint(UTM(info["zone"]), inter_easting.min(), inter_northing.max())
    return nw_coord, map_array

def loadElevationMap(file_path, maxSlope=15, planet='Earth', nw_corner=None, se_corner=None, desired_res=None,
                     no_val=-10000):
    '''
	Creates a EnvironmentalModel object from either a geoTiff file or a text file.

	Issue: computer sometimes freezes whenever loading a very large geoTIFF file (1GB+)
	Solution: optional inputs: NWCorner and SE corner - GeoPoints

	Using the parameter desiredRes doesn't work very well if there are a significant number of
	"unknown" points (usually denoted by a placeholder like -10000).
	'''
    dataset, info = loadElevationsLite(file_path)
    width, height, resolution = info["width"], info["height"], info["resolution"]

    map_nw_corner = info["nw_geo_point"]
    XY = Cartesian(map_nw_corner, resolution)
    map_se_corner = GeoPoint(XY, width, height)

    selection_nw_corner = map_nw_corner if nw_corner is None else nw_corner
    selection_se_corner = map_se_corner if se_corner is None else se_corner

    map_box = GeoPolygon([map_nw_corner, map_se_corner]).envelope
    selection_box = GeoEnvelope(selection_nw_corner, selection_se_corner).envelope
    intersection_box = map_box.intersection(selection_box)

    inter_easting, inter_northing = np.array(intersection_box.bounds).reshape((2, 2)).transpose()
    intersection_box_geo = GeoPolygon(UTM(info["zone"]), inter_easting, inter_northing)
    inter_x, inter_y = intersection_box_geo.to(XY)

    x_offset, max_x = inter_x
    max_y, y_offset = inter_y
    x_size = max_x - x_offset
    y_size = max_y - y_offset

    buf_x = None
    buf_y = None
    if desired_res:
        buf_x = int(x_size * resolution / desired_res)
        buf_y = int(y_size * resolution / desired_res)
    else:
        desired_res = resolution

    band = dataset.GetRasterBand(1)
    map_array = band.ReadAsArray(x_offset, y_offset, x_size, y_size, buf_x, buf_y).astype(np.float)
    nw_coord = GeoPoint(UTM(info["zone"]), inter_easting.min(), inter_northing.max())

    return EnvironmentalModel(map_array, desired_res, maxSlope, nw_coord, planet)