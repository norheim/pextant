import re
from pextant.lib.geoshapely import *
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

        super(GDALMesh, self).__init__(nw_geo_point, width, height, resolution, dataset)

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
        map_array_clean = ma.masked_array(map_array, np.isnan(map_array)).filled(-99999)
        map_array_clean = ma.masked_array(map_array_clean, map_array_clean < 0)
        # TODO: this hack needs explanation
        nw_coord_hack = GeoPoint(UTM(zone), inter_easting.min(), inter_northing.max()).to(XY)
        nw_coord = GeoPoint(XY, nw_coord_hack[0], nw_coord_hack[1])
        return EnvironmentalModel(nw_coord, x_size, y_size, desired_res, map_array_clean, self.planet,
                                  maxSlope, self, x_offset, y_offset)


class EnvironmentalModel(Mesh):
    """
    This class ultimately represents an elevation map + all of the traversable spots on it.
    """

    def __init__(self, nw_geo_point, width, height, resolution, dataset, planet, maxSlope=35, parentMesh=None, xoff=0, yoff=0):
        super(EnvironmentalModel, self).__init__(nw_geo_point, width, height, resolution, dataset, planet,
                                                 parentMesh, xoff, yoff)
        self.parent = self.parentMesh
        self.numRows, self.numCols = dataset.shape
        self.slopes = None
        self.ismissingdata = self.dataset.mask
        self.obstacles = np.zeros(dataset.shape, dtype=bool)  # obstacles is a matrix with boolean values for passable squares
        self.planet = planet
        self.ROW_COL = Cartesian(nw_geo_point, resolution, reverse=True)
        self.COL_ROW = Cartesian(nw_geo_point, resolution)
        self.special_obstacles = set()  # a list of coordinates of obstacles are not identified by the slope
        self.searchKernel = SearchKernel()
        self.setSlopes()
        #TODO: make max slope a once only argument (right now it gets passed along several times)
        self.maxSlopeObstacle(maxSlope)

    def getMeshElement(self, geo_point):
        row, col = geo_point.to(self.ROW_COL)
        return self._getMeshElement(row, col)

    def _getMeshElement(self, row, col):
        if self._inBounds(row, col):
            return MeshElement(row, col, self)
        else:
            raise IndexError("The location (%s, %s) is out of bounds" % row, col)

    def getNeighbours(self, mesh_element):
        state = mesh_element.getCoordinates()
        kernel = self.searchKernel
        offset = kernel.getKernel()
        potential_neighbours = offset + state
        children = [MeshElement(row, col, self) for (row, col) in potential_neighbours
                    if self._isPassable(row, col)]
        return MeshCollection(children)

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

    def _isPassable(self, row, col):
        if self._hasdata(row,col):
            return self.obstacles[row][col]
        else:
            return False

    def isPassable(self, coordinates):
        # determines if coordinates can be passed through
        row, col = self.convertToRowCol(coordinates)
        return self._isPassable(row, col)

    def convertToRowCol(self, coordinates):
        if isinstance(coordinates, GeoPoint):
            return coordinates.to(self.ROW_COL)
        else:
            return coordinates


def loadElevationMap(fullPath, maxSlope=35, nw_corner=None, se_corner=None, desiredRes=None):
    geoenvelope = GeoEnvelope(nw_corner, se_corner)
    maxSlope=35 #TODO: need to fix this
    dem = GDALMesh(fullPath)
    return dem.loadMapSection(geoenvelope, maxSlope=maxSlope, desired_res=desiredRes)


if __name__ == '__main__':
    from pextant.analysis.loadWaypoints import JSONloader
    from pextant.ExplorerModel import Astronaut
    from pextant.solvers.astarSEXTANT import ExplorerCost, ExpandViz, fullSearch
    hi_low = GDALMesh('../data/maps/HI_lowqual_DEM.tif')
    waypoints = JSONloader.from_file('../data/waypoints/HI_13Nov16_MD7_A.json').get_waypoints()
    geoenvelope = waypoints.geoEnvelope()
    env_model = hi_low.loadMapSection(geoenvelope)
    nw_corner, se_corner = geoenvelope.getBounds()
    env_model2 = loadElevationMap('../data/maps/HI_lowqual_DEM.tif',
                                  nw_corner=nw_corner, se_corner=se_corner)
    astronaut = Astronaut(80)
    cost_function = ExplorerCost(astronaut, env_model, "Energy")
    fullSearch(waypoints, env_model2, cost_function, viz=ExpandViz(env_model.numRows, env_model.numCols))
