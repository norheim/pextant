from geoshapely import *
from osgeo import gdal, osr
import re


class Mesh(object):
    def __init__(self, nw_geo_point, width, height, resolution, dataset, planet='Earth',
                 parentMesh=None, xoff=0, yoff=0):
        self.nw_geo_point = nw_geo_point
        self.width = width
        self.height = height
        self.resolution = resolution
        self.parentMesh = parentMesh
        self.xoff = xoff
        self.yoff = yoff
        self.dataset = dataset
        self.planet = planet

    def __str__(self):
        return 'height: %s \nwidth: %s \nresolution: %s \nnw corner: %s' % \
              (self.height, self.width, self.resolution, str(self.nw_geo_point))

class GDALMesh(Mesh):
    def __init__(self, file_path):
        gdal.UseExceptions()
        dataset = gdal.Open(file_path)
        width = dataset.RasterXSize
        height = dataset.RasterYSize

        dataset_info = dataset.GetGeoTransform()
        nw_easting = dataset_info[0]
        nw_northing = dataset_info[3]
        resolution = dataset_info[1]

        proj = dataset.GetProjection()
        srs = osr.SpatialReference(wkt=proj)
        projcs = srs.GetAttrValue('projcs')  # "NAD83 / UTM zone 5N"...hopefully
        regex_result = re.search('zone\s(\d+)(\w)', projcs)
        zone_number = regex_result.group(1)  # zone letter is group(2)
        nw_geo_point = GeoPoint(UTM(zone_number), nw_easting, nw_northing)

        super(GDALMesh, self).__init__(nw_geo_point, width, height, resolution, dataset)

    def loadMapSection(self, geo_envelope=None, desired_res=None):
        """


        :param geo_envelope:
        :type geo_envelope pextant.geoshapely.GeoEnvelope
        :param desired_res:
        :return:
        """
        map_nw_corner, zone = self.nw_geo_point, self.nw_geo_point.utm_reference.proj_param["zone"]
        dataset = self.dataset

        XY = Cartesian(map_nw_corner, self.resolution)
        map_se_corner = GeoPoint(XY, self.width, self.height)

        if geo_envelope is not None:
            selection_nw_corner, selection_se_corner = geo_envelope.getBounds()
        else:
            selection_nw_corner, selection_se_corner = map_nw_corner, map_se_corner

        map_box = GeoPolygon([map_nw_corner, map_se_corner]).envelope
        selection_box = GeoEnvelope(selection_nw_corner, selection_se_corner).envelope
        intersection_box = map_box.intersection(selection_box)

        inter_easting, inter_northing = np.array(intersection_box.bounds).reshape((2, 2)).transpose()
        intersection_box_geo = GeoPolygon(UTM(zone), inter_easting, inter_northing) # could change to UTM_AUTO later
        inter_x, inter_y = intersection_box_geo.to(XY)

        x_offset, max_x = inter_x
        max_y, y_offset = inter_y
        x_size = max_x - x_offset
        y_size = max_y - y_offset

        buf_x = None
        buf_y = None
        if desired_res:
            buf_x = int(x_size * self.resolution / desired_res)
            buf_y = int(y_size * self.resolution / desired_res)
        else:
            desired_res = self.resolution

        band = dataset.GetRasterBand(1)
        map_array = band.ReadAsArray(x_offset, y_offset, x_size, y_size, buf_x, buf_y).astype(np.float)
        nw_coord = GeoPoint(UTM(zone), inter_easting.min(), inter_northing.max())
        return EnvironmentalModel(nw_coord, x_size, y_size, desired_res, map_array, self.planet,
                                  self, x_offset, y_offset)

class EnvironmentalModel(Mesh):
    """
    This class ultimately represents an elevation map + all of the traversable spots on it.

    Public functions:

    setMaxSlope(slope) - sets the maximum slope that can be traversed
    setObstacle(coordinates), eraseObstacle(coordinates) - set or erase an obstacle at certain coordinates
    getElevation(coordinates), getSlope(coordinates) - get Elevation or Slope at certain coordinates
    isPassable(coordinates) - determines if a certain coordinate can be traversed
    convertToRowCol(coordinates), convertToUTM(coordinates), convertToLatLong(coordinates)
    - converts from one coordinate system to another
    """
    def __init__(self, nw_geo_point, width, height, resolution, dataset, planet, parentMesh=None, xoff=0, yoff=0):
        super(EnvironmentalModel, self).__init__(nw_geo_point, width, height, resolution, dataset, planet,
                                                 parentMesh, xoff, yoff)
        elevation_map = self.dataset
        [gx, gy] = np.gradient(elevation_map, resolution, resolution)
        self.slopes = np.degrees(
            np.arctan(np.sqrt(np.add(np.square(gx), np.square(gy)))))  # Combining for now for less RAM usage
        self.numRows, self.numCols = elevation_map.shape
        self.obstacles = None  # obstacles is a matrix with boolean values for passable squares
        self.planet = planet
        self.ROW_COL = Cartesian(nw_geo_point, resolution, reverse=True)
        self.special_obstacles = set()  # a list of coordinates of obstacles are not identified by the slope

    def getGravity(self):
        if self.planet == 'Earth':
            return 9.81
        elif self.planet == 'Moon':
            return 1.622

    def maxSlopeObstacle(self, maxSlope):
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
        return self.dataset[row][col]

    def getSlope(self, coordinates):
        row, col = self.convertToRowCol(coordinates)
        return self.slopes[row][col]

    def setNoVal(self, noValResult):
        for i, row in enumerate(self.dataset):
            for j, item in enumerate(row):
                if item == noValResult:
                    self.dataset[i][j] = None
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