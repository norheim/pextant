import numpy as np
import numpy.matlib as matlib
from pextant.lib.geoshapely import GeoPoint, Cartesian, XY
from scipy.interpolate import interp2d, NearestNDInterpolator, RegularGridInterpolator
from skimage.draw import circle

class GeoMesh(object):
    def __init__(self, nw_geo_point, dataset, resolution=1, planet='Earth',
                 parent_mesh=None, xoff=0, yoff=0):
        self.dataset = dataset.data
        self.x_size = dataset.x_size
        self.y_size = dataset.y_size
        self.shape = dataset.shape
        self.size = dataset.size

        self.local_coordinates = XY(nw_geo_point, resolution)
        self.nw_geo_point = nw_geo_point
        self.se_geo_point = GeoPoint(Cartesian(nw_geo_point, resolution), self.x_size, self.y_size)
        self.resolution = resolution
        self.planet = planet if parent_mesh == None else parent_mesh.planet

        # data points that make it relative to a parent mesh
        self.parent_mesh = parent_mesh
        self.xoff = xoff
        self.yoff = yoff

    @classmethod
    def from_parent(cls, parent, **kwargs):
        return cls(parent.nw_geo_point, parent.dataset, parent.resolution, parent_mesh=parent.parent_mesh,
                   xoff=parent.xoff, yoff=parent.yoff, **kwargs)

    def localpoint(self, geopoint):
        return geopoint.to(self.local_coordinates)

    def __str__(self):
        return 'height: %s \nwidth: %s \nresolution: %s \nnw corner: %s' % \
              (self.y_size, self.x_size, self.resolution, str(self.nw_geo_point))

class Dataset(object):
    '''
    This is a wrapper that gives gdal datasets a shape property, similar to numpy arrays
    '''
    def __init__(self, data, row_size, col_size):
        self.y_size = row_size
        self.x_size = col_size
        self.shape = (row_size, col_size)
        self.size = row_size*col_size
        self.data = data

    def __str__(self):
        return 'height: %s \nwidth: %s ' % \
              (self.y_size, self.x_size)

class InterpolatingDataset(Dataset):
    def __init__(self, data, row_size, col_size):
        super(InterpolatingDataset, self).__init__(data, row_size, col_size)
        self.interpolator = self._grid_interpolator_initializer()

    @classmethod
    def from_np(cls, numpy_array):
        y_size, x_size = numpy_array.shape
        return cls(numpy_array, y_size, x_size)

    # should be overwritten if needed, which is why it's its own function outside of __init__
    def _grid_interpolator_initializer(self):
        x_axis = np.arange(self.x_size)
        y_axis = np.arange(self.y_size)
        return RegularGridInterpolator((y_axis, x_axis), self.data)

    # interpolation should be defined by children classes
    def get_datapoint(self, localpoint):
        return self.interpolator(localpoint)

class GridMesh(GeoMesh):
    def __init__(self, *arg, **kwargs):
        super(GridMesh, self).__init__(*arg, **kwargs)
        self.ROW_COL = Cartesian(self.nw_geo_point, self.resolution, reverse=True)
        self.COL_ROW = Cartesian(self.nw_geo_point, self.resolution)

def coordinate_transform(func):
    def decorated(self, point):
        return func(self, self.convert_coordinates(point))
    return decorated

class EnvironmentalModel(GeoMesh):
    """
    This class ultimately represents an elevation map + all of the traversable spots on it.
    """
    def __init__(self, nw_geo_point, dataset, resolution=1, planet='Earth',
                 parent_mesh=None, xoff=0, yoff=0, maxSlope=35, cached=False):
        super(EnvironmentalModel, self).__init__(nw_geo_point, dataset, resolution, planet,
                 parent_mesh, xoff, yoff)
        self.maxSlope = maxSlope
        self.cached = cached
        self.dataset_unmasked = self.dataset.filled(0)
        self.isvaliddata = np.logical_not(self.dataset.mask)
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