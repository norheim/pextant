import numpy as np
import numpy.matlib as matlib
from pextant.lib.geoshapely import GeoPoint, GeoPolygon, GeoEnvelope, Cartesian, XY, UTM
from scipy.interpolate import NearestNDInterpolator, RegularGridInterpolator, griddata
from scipy.ndimage.interpolation import zoom
from skimage.draw import circle

class GeoMesh(object):
    def __init__(self, nw_geo_point, dataset, planet='Earth',
                 parent_mesh=None, xoff=0, yoff=0):
        self.dataset = dataset
        self.data = dataset.data_container
        self.x_size = dataset.x_size
        self.y_size = dataset.y_size
        self.shape = dataset.shape
        self.size = dataset.size
        self.resolution = dataset.resolution

        self.local_coordinates = XY(nw_geo_point, self.resolution)
        self.nw_geo_point = nw_geo_point
        self.se_geo_point = GeoPoint(Cartesian(nw_geo_point, self.resolution), self.x_size, self.y_size)
        self.planet = planet if parent_mesh == None else parent_mesh.planet

        # data points that make it relative to a parent mesh, but not really currently in use
        self.parent_mesh = parent_mesh
        self.xoff = xoff
        self.yoff = yoff

    @classmethod
    def from_parent(cls, parent, **kwargs):
        return cls(parent.nw_geo_point, parent.dataset, parent_mesh=parent.parent_mesh,
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
    def __init__(self, data, row_size, col_size, resolution):
        self.y_size = row_size
        self.x_size = col_size
        self.shape = (row_size, col_size)
        self.size = row_size*col_size
        self.data_container = data
        self.resolution = resolution

    def __str__(self):
        return 'height: %s \nwidth: %s ' % \
              (self.y_size, self.x_size)

class InterpolatingDataset(Dataset):
    def __init__(self, data, row_size, col_size, resolution=1.):
        super(InterpolatingDataset, self).__init__(data, row_size, col_size, resolution)
        self.interpolator = self._grid_interpolator_initializer()

    @classmethod
    def from_np(cls, numpy_array, resolution=1.):
        y_size, x_size = numpy_array.shape
        return cls(numpy_array, y_size, x_size, resolution=resolution)

    # should be overwritten if needed, which is why it's its own function outside of __init__
    def _grid_interpolator_initializer(self):
        x_axis = np.arange(self.x_size)
        y_axis = np.arange(self.y_size)
        return RegularGridInterpolator((y_axis, x_axis), self.data_container)

    def downsample(self, resolution):
        return InterpolatingDataset.from_np(zoom(self.data_container, self.resolution/resolution))

    # interpolation should be defined by children classes
    def get_datapoint(self, localpoint):
        return self.interpolator(localpoint)


class NpDataset(np.ndarray):
    # Wraps around Interpolating Dataset aswell
    def __new__(cls, input_array, resolution=1.0):
        # Input array is an already formed ndarray instance
        # We first cast to be our class type
        obj = np.asarray(input_array).view(cls)
        # add the new attribute to the created instance
        obj.resolution = resolution
        obj.interp = InterpolatingDataset.from_np(input_array, resolution=resolution)
        # Finally, we must return the newly created object:
        return obj

    def downsample(self, resolution):
        return NpDataset(zoom(self.interp.data_container, float(self.interp.resolution) / resolution), resolution)

    def subsection(self, xoff, yoff, xsize, ysize, resolution=None):
        x2 = xoff+xsize
        y2 = yoff+ysize
        sub =  NpDataset(self.interp.data_container[xoff:x2, yoff:y2])
        if resolution == None:
             return sub
        else:
            return sub.downsample(resolution)

    def __getattr__(self, item):
        try:
            return InterpolatingDataset.__getattribute__(self, item)
        except AttributeError:
            return getattr(self.interp, item)

    def __array_finalize__(self, obj):
        # see InfoArray.__array_finalize__ for comments
        if obj is None: return
        resolution =  getattr(obj, "resolution", None)
        self.interp = InterpolatingDataset.from_np(self, resolution)

    def __repr__(self):
        return np.array(self.interp.data_container).__repr__()


class NearestInterpolatorDataset(InterpolatingDataset):
    def __init__(self, *arg, **kwargs):
        super(NearestInterpolatorDataset, self).__init__(*arg, **kwargs)

    def _grid_interpolator_initializer(self):
        x_axis = np.arange(self.x_size)
        y_axis = np.arange(self.y_size)
        return NearestNDInterpolator((y_axis, x_axis), self.data_container)

class GridMesh(GeoMesh):
    def __init__(self, *arg, **kwargs):
        super(GridMesh, self).__init__(*arg, **kwargs)
        self.ROW_COL = Cartesian(self.nw_geo_point, self.resolution, reverse=True)
        self.COL_ROW = Cartesian(self.nw_geo_point, self.resolution)

    def subsection(self, geo_envelope=None, desired_res=None):
        """
                :param geo_envelope:
                :type geo_envelope pextant.geoshapely.GeoEnvelope
                :param desired_res:
                :return:
        """
        map_nw_corner, map_se_corner  = self.nw_geo_point, GeoPoint(self.COL_ROW, self.x_size, self.y_size)

        if geo_envelope is not None:
            selection_nw_corner, selection_se_corner = geo_envelope.getBounds()
        else:
            selection_nw_corner, selection_se_corner = map_nw_corner, map_se_corner

        map_box = GeoPolygon([map_nw_corner, map_se_corner]).envelope
        selection_box = GeoEnvelope(selection_nw_corner, selection_se_corner).envelope
        intersection_box = map_box.intersection(selection_box)

        inter_easting, inter_northing = np.array(intersection_box.bounds).reshape((2, 2)).transpose()
        intersection_box_geo = GeoPolygon(self.nw_geo_point.utm_reference, inter_easting, inter_northing)  # could change to UTM_AUTO later
        inter_x, inter_y = intersection_box_geo.to(self.COL_ROW)

        x_offset, max_x = inter_x
        max_y, y_offset = inter_y
        # TODO: need to explain the +1, comes from hack. Also, doesnt work when selecting full screen
        x_size = max_x - x_offset + 1
        y_size = max_y - y_offset + 1
        if geo_envelope is None:
            x_size -= 1
            y_size -= 1

        dataset_clean = self.dataset.subsection(x_offset, y_offset, x_size, y_size, desired_res)

        # TODO: this hack needs explanation
        nw_coord_hack = GeoPoint(self.nw_geo_point.utm_reference, inter_easting.min(), inter_northing.max())\
            .to(self.COL_ROW)
        nw_coord = GeoPoint(self.COL_ROW, nw_coord_hack[0], nw_coord_hack[1])
        meta_mesh = GridMesh(nw_coord, dataset_clean, parent_mesh=self,
                             xoff=x_offset, yoff=y_offset)
        return meta_mesh

def coordinate_transform(func):
    def decorated(self, point):
        return func(self, self.convert_coordinates(point))
    return decorated

class EnvironmentalModel(GeoMesh):
    """
    This class ultimately represents an elevation map + all of the traversable spots on it.
    """
    def __init__(self, nw_geo_point, dataset, planet='Earth',
                 parent_mesh=None, xoff=0, yoff=0, maxSlope=35, kernel_size=3, kernel_type="square",
                 cached=False):
        super(EnvironmentalModel, self).__init__(nw_geo_point, dataset, planet,
                 parent_mesh, xoff, yoff)
        self.maxSlope = maxSlope
        self.cached = cached
        self.kernel_type = kernel_type
        self.kernel_size = kernel_size
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
        return self._getNeighbours(mesh_element.state)

    @coordinate_transform
    def get_neighbours(self, geo_coordinates):
        return self._getNeighbours(geo_coordinates)

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
    def __init__(self, kernelrange = 3, type="square"):
        searchvector = range(-(kernelrange / 2), kernelrange / 2 + 1)
        col_off = matlib.repmat(searchvector, len(searchvector), 1)
        row_off = np.transpose(col_off)
        col_off_clean = np.delete(col_off.flatten(), (kernelrange**2 - 1)/2)
        row_off_clean = np.delete(row_off.flatten(), (kernelrange**2 - 1) / 2)
        self.length = kernelrange**2
        self.col_off = col_off_clean
        self.row_off = row_off_clean
        self.kernelrange = kernelrange
        self.type = type
        self.typemap = dict(square=self.get_square_kernel, circular=self.get_circular_kernel)
        self.kernel = np.array([row_off_clean, col_off_clean]).transpose()

    def getKernel(self):
        return self.typemap[self.type]()

    def get_square_kernel(self):
        return self.kernel

    def get_circular_kernel(self):
        kernel = np.array(circle(0, 0, int((self.kernelrange+1)/2+1))).transpose()
        center_removed = np.delete(kernel, np.where(np.logical_and(kernel[:, 0] == 0, kernel[:, 1] == 0))[0][0], 0)
        return center_removed