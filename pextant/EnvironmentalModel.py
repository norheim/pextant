import re
from pextant.lib.geoshapely import *
from pextant.lib.geoutils import filled_circle
from pextant.MeshModel import Mesh, MeshElement, MeshCollection, SearchKernel

from osgeo import gdal, osr
import numpy.ma as ma

class GDALMesh(Mesh):
    """
    This class should be used for loading all GDAL terrains, and any subset thereof
    """

    def __init__(self, file_path):
        self.file_path = file_path
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

        super(GDALMesh, self).__init__(nw_geo_point, dataset, resolution, width=width, height=height)

    def loadMapSection(self, geo_envelope=None, maxSlope=35, desired_res=None):
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
        intersection_box_geo = GeoPolygon(UTM(zone), inter_easting, inter_northing)  # could change to UTM_AUTO later
        inter_x, inter_y = intersection_box_geo.to(XY)

        x_offset, max_x = inter_x
        max_y, y_offset = inter_y
        # TODO: need to explain the +1, comes from hack. Also, doesnt work when selecting full screen
        x_size = max_x - x_offset + 1
        y_size = max_y - y_offset + 1
        if geo_envelope is None:
            x_size -= 1
            y_size -= 1

        buf_x = None
        buf_y = None
        if desired_res:
            buf_x = int(x_size * self.resolution / desired_res)
            buf_y = int(y_size * self.resolution / desired_res)
        else:
            desired_res = self.resolution

        band = dataset.GetRasterBand(1)
        map_array = band.ReadAsArray(x_offset, y_offset, x_size, y_size, buf_x, buf_y).astype(np.float)

        # TODO: this hack needs explanation
        nw_coord_hack = GeoPoint(UTM(zone), inter_easting.min(), inter_northing.max()).to(XY)
        nw_coord = GeoPoint(XY, nw_coord_hack[0], nw_coord_hack[1])
        return EnvironmentalModel(nw_coord, map_array, desired_res, self.planet,
                                  maxSlope, self, x_offset, y_offset)


class EnvironmentalModel(Mesh):
    """
    This class ultimately represents an elevation map + all of the traversable spots on it.
    """

    def __init__(self, nw_geo_point, dataset, resolution, planet='Earth', maxSlope=35, parentMesh=None, xoff=0, yoff=0):
        dataset_clean = ma.masked_array(dataset, np.isnan(dataset)).filled(-99999)
        dataset_clean = ma.masked_array(dataset_clean, dataset_clean < 0)
        super(EnvironmentalModel, self).__init__(nw_geo_point, dataset_clean, resolution, planet,
                                                 parentMesh, xoff, yoff)
        self.numRows, self.numCols = dataset.shape
        self.slopes = None
        self.ismissingdata = self.dataset.mask
        #TODO: should obstacles be a set? since its a sparse matrix
        self.obstacles = np.zeros(dataset.shape, dtype=bool)  # obstacles is a matrix with boolean values for passable squares
        self.special_obstacles = set()  # a list of coordinates of obstacles are not identified by the slope
        self.searchKernel = SearchKernel()
        self.setSlopes()
        self.maxSlope = maxSlope
        #TODO: make max slope a once only argument (right now it gets passed along several times)
        self.maxSlopeObstacle(maxSlope)

    def getMeshElement(self, coordinates):
        row, col = self.convertToRowCol(coordinates)
        return self._getMeshElement(row, col)

    def _getMeshElement(self, row, col):
        if self._inBounds(row, col):
            return MeshElement(row, col, self)
        else:
            raise IndexError("The location (%s, %s) is out of bounds" % row, col)

    def getNeighbours(self, mesh_element):
        return self._getNeighbours(mesh_element.getCoordinates())

    def _getNeighbours(self, rowcol):
        state = np.array(rowcol)
        kernel = self.searchKernel
        offset = kernel.getKernel()
        potential_neighbours = offset + state
        passable_neighbours = np.array(filter(self._isPassable, potential_neighbours))
        return MeshCollection(self, passable_neighbours)

    def getGravity(self):
        if self.planet == 'Earth':
            return 9.81
        elif self.planet == 'Moon':
            return 1.622

    def setSlopes(self):
        [gx, gy] = np.gradient(self.dataset, self.resolution, self.resolution)
        self.slopes = np.degrees(
            np.arctan(np.sqrt(np.add(np.square(gx), np.square(gy)))))  # Combining for now for less RAM usage

    def maxSlopeObstacle(self, maxSlope):
        self.obstacles = self.slopes <= maxSlope
        for obstacle in self.special_obstacles:
            self.setObstacle(obstacle)  # "special obstacles" still should not be traversable

    def setObstacle(self, coordinates):
        row, col = self.convertToRowCol(coordinates)  # of the form (row, column)
        self.obstacles[row][col] = False
        if coordinates not in self.special_obstacles:
            self.special_obstacles.add(coordinates)

    def setRadialKeepOutZone(self, center, radius):
        circlex, circley = filled_circle(self.ROW_COL, center, radius)
        self.obstacles[circlex, circley] = 1

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

    def in_bounds(self, coordinates):
        row, col = self.convertToRowCol(coordinates)
        return self._inBounds(row, col)

    def has_data(self, coordinates):
        row, col = self.convertToRowCol(coordinates)
        return self._hasdata(row, col)

    def _inBounds(self, row, col):
        # determines if a coordinate is within the boundaries of the environmental model
        return (0 <= row < self.numRows) and (0 <= col < self.numCols)

    def _hasdata(self, row, col):
        return self._inBounds(row, col) and not self.ismissingdata[row, col] #use lazy evaluation in case out of bounds

    def _isPassable(self, rowcol):
        row, col = rowcol
        return self._hasdata(row,col) and self.obstacles[row, col]

    def isPassable(self, coordinates):
        # determines if coordinates can be passed through
        row, col = self.convertToRowCol(coordinates)
        return self._isPassable((row, col))

    def convertToRowCol(self, coordinates):
        if isinstance(coordinates, GeoPoint):
            return coordinates.to(self.ROW_COL)
        else:
            return coordinates

    def cache_neighbours(self):
        s = dict()
        for i in range(self.numRows):
            for j in range(self.numCols):
                s[(i,j)] = self._getNeighbours((i,j))
        return s

def loadElevationMap(fullPath, maxSlope=35, nw_corner=None, se_corner=None, desiredRes=None):
    geoenvelope = GeoEnvelope(nw_corner, se_corner)
    maxSlope=35 #TODO: need to fix this
    dem = GDALMesh(fullPath)
    return dem.loadMapSection(geoenvelope, maxSlope=maxSlope, desired_res=desiredRes)


if __name__ == '__main__':
    from pextant.settings import AMES_DEM
    import cProfile
    ames_em = GDALMesh('../data/maps/Ames/Ames.tif').loadMapSection()
    cProfile.run('ames_em.cache_neighbours()')
    print(ames_em.numCols*ames_em.numRows)
