import numpy as np
import numpy.matlib as matlib
import json
from pextant.lib.geoshapely import GeoPoint, Cartesian
from skimage.draw import circle

class MetaMesh(object):
    def __init__(self, nw_geo_point, dataset, resolution=1, planet='Earth',
                 parent_mesh=None, xoff=0, yoff=0):
        y_size, x_size = dataset.shape  #y is first in case dataset is a numpy array
        self.x_size = x_size
        self.y_size = y_size
        self.shape = dataset.shape
        self.size = dataset.size
        self.nw_geo_point = nw_geo_point
        self.se_geo_point = GeoPoint(Cartesian(nw_geo_point, resolution), self.x_size, self.y_size)
        self.dataset = dataset
        self.resolution = resolution
        self.parent_mesh = parent_mesh
        self.planet = planet if self.parent_mesh == None else parent_mesh.planet
        self.xoff = xoff
        self.yoff = yoff

    def jsonify(self):
        return json.dumps(self.__dict__)

    def __str__(self):
        return 'height: %s \nwidth: %s \nresolution: %s \nnw corner: %s' % \
              (self.y_size, self.x_size, self.resolution, str(self.nw_geo_point))

class GridMesh(MetaMesh):
    def __init__(self, *arg, **kwargs):
        super(GridMesh, self).__init__(*arg, **kwargs)
        self.ROW_COL = Cartesian(self.nw_geo_point, self.resolution, reverse=True)
        self.COL_ROW = Cartesian(self.nw_geo_point, self.resolution)

def coordinate_transform(func):
    def decorated(self, point):
        return func(self, self.convert_coordinates(point))
    return decorated

class EnvironmentalModel(MetaMesh):
    """
    This class ultimately represents an elevation map + all of the traversable spots on it.
    """
    def __init__(self, nw_geo_point, dataset, resolution=1, planet='Earth',
                 parent_mesh=None, xoff=0, yoff=0, maxSlope=35, cached=False):
        super(EnvironmentalModel, self).__init__(nw_geo_point, dataset, resolution, planet,
                 parent_mesh, xoff, yoff)
        self.maxSlope = maxSlope
        self.cached = cached
        self.dataset_unmasked = dataset.filled(0)
        self.isvaliddata = np.logical_not(dataset.mask)
        self.slopes = []
        self.obstacles = []  # obstacles is a list with boolean values for non-passable squares
        self.passable = []
        self.special_obstacles = set()  # a list of coordinates of obstacles are not identified by the slope
        self.setSlopes()
        #TODO: make max slope a once only argument (right now it gets passed along several times)
        self.maxSlopeObstacle(self.maxSlope)

    def convert_coordinates(self, geo_coordinates): pass

    def setSlopes(self): pass

    def getSlope(self, mesh_coordinates):
        return self.slopes[mesh_coordinates]

    def getElevations(self, mesh_coordinates): pass

    def getGravity(self):
        if self.planet == 'Earth':
            return 9.81
        elif self.planet == 'Moon':
            return 1.622

    def maxSlopeObstacle(self, maxSlope):
        self.obstacles.extend(np.where(self.slopes > maxSlope)[0])

    @coordinate_transform
    def getMeshElement(self, geo_coordinates):
        return self._getMeshElement(geo_coordinates)

    def _getMeshElement(self, mesh_coordinates): pass

    def getMeshEltNeighbours(self, mesh_element):
        self._getNeighbours(mesh_element.state)

    def _getNeighbours(self, mesh_coordinates): pass

    # obstacle functions
    @coordinate_transform
    def eraseObstacle(self, geo_coordinates):
        self.obstacles.remove(geo_coordinates)

    @coordinate_transform
    def setObstacle(self, geo_coordinates):
        self.obstacles.append(geo_coordinates)

    def setRadialKeepOutZone(self, center, radius): pass

    @coordinate_transform
    def in_bounds(self, geo_coordinates):
        return self._inBounds(geo_coordinates)

    def _inBounds(self, mesh_coordinates): pass

    @coordinate_transform
    def has_data(self, geo_coordinates):
        return self._hasdata(geo_coordinates)

    def _hasdata(self, mesh_coordinates): pass

    @coordinate_transform
    def isPassable(self, geo_coordinates):
        # determines if coordinates can be passed through
        return self._isPassable(geo_coordinates)

    def _isPassable(self, mesh_coordinates):
        return self._hasdata(mesh_coordinates) and mesh_coordinates not in self.obstacles

class SearchKernel(object):
    def __init__(self, kernelrange = 3):
        searchvector = range(-(kernelrange / 2), kernelrange / 2 + 1)
        col_off = matlib.repmat(searchvector, len(searchvector), 1)
        row_off = np.transpose(col_off)
        col_off_clean = np.delete(col_off.flatten(), (kernelrange**2 - 1)/2)
        row_off_clean = np.delete(row_off.flatten(), (kernelrange**2 - 1) / 2)
        self.length = kernelrange**2
        self.col_off = col_off_clean
        self.row_off = row_off_clean
        self.kernelrange = kernelrange
        self.kernel = np.array([row_off_clean, col_off_clean]).transpose()

    def getKernel(self):
        return self.kernel

    def getCircularKernel(self):
        return np.array(circle(0, 0, 6)).transpose()