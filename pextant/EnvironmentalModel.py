import re

import numpy.ma as ma
from osgeo import gdal, osr

from pextant.lib.geoshapely import *
from pextant.lib.geoutils import filled_grid_circle
from pextant.mesh.abstractmesh import GeoMesh, GridMesh, EnvironmentalModel, \
    SearchKernel, coordinate_transform, Dataset, NpDataset
from pextant.mesh.abstractcomponents import MeshCollection
from pextant.mesh.concretecomponents import MeshElement

class GDALDataset(Dataset):
    '''
    This is a wrapper that gives gdal datasets a shape property, similar to numpy arrays
    '''
    def __init__(self, dataset, row_size, col_size, resolution):
        super(GDALDataset, self).__init__(dataset, row_size, col_size, resolution)
        self.raster = dataset

    # override interpolator since the data set is just a shell
    def _grid_interpolator_initializer(self):
        return lambda : None

    def subsection(self, x_offset, y_offset, x_size, y_size, desired_res=None):
        buf_x = None
        buf_y = None
        if desired_res:
            buf_x = int(x_size * self.resolution / desired_res)
            buf_y = int(y_size * self.resolution / desired_res)
        else:
            desired_res = self.resolution

        band = self.raster.GetRasterBand(1)
        map_array = band.ReadAsArray(x_offset, y_offset, x_size, y_size, buf_x, buf_y).astype(np.float)
        dataset_clean = ma.masked_array(map_array, np.isnan(map_array)).filled(-99999)
        dataset_clean = NpDataset(ma.masked_array(dataset_clean, dataset_clean < 0), desired_res)
        return dataset_clean


class GDALMesh(GridMesh):
    """
    This class should be used for loading all GeoTiff terrains, and any subset thereof
    """

    def __init__(self, file_path):
        self.file_path = file_path
        gdal.UseExceptions()
        dataset = gdal.Open(file_path)
        x_size = dataset.RasterXSize
        y_size = dataset.RasterYSize

        dataset_info = dataset.GetGeoTransform()
        nw_easting = dataset_info[0]
        nw_northing = dataset_info[3]
        resolution = dataset_info[1]
        dataset_wrapped = GDALDataset(dataset, row_size=y_size, col_size=x_size,
                                      resolution=resolution)

        proj = dataset.GetProjection()
        srs = osr.SpatialReference(wkt=proj)
        projcs = srs.GetAttrValue('projcs')  # "NAD83 / UTM zone 5N"...hopefully
        regex_result = re.search('zone(\s|\_)(\d+)(\w)', projcs, flags=re.IGNORECASE)
        if regex_result:
            zone_number = regex_result.group(2)  # zone letter is group(2)
            nw_geo_point = GeoPoint(UTM(zone_number), nw_easting, nw_northing)
        else:
            nw_geo_point = GeoPoint(UTM(0), 0, 0)

        super(GDALMesh, self).__init__(nw_geo_point, dataset_wrapped)

    def loadSubSection(self, geo_envelope=None, desired_res=None, **kwargs):
        #kwargs is reserved for max slope argument
        sub_mesh = self.subsection(geo_envelope, desired_res)
        return GridMeshModel.from_parent(sub_mesh, **kwargs)


class GridMeshModel(EnvironmentalModel):
    def __init__(self, *arg, **kwargs):
        super(GridMeshModel, self).__init__(*arg, **kwargs)
        self.dataset_unmasked = self.data.filled(0) if isinstance(self.data, np.ma.core.MaskedArray) else self.data
        self.isvaliddata = np.logical_not(self.data.mask)  if isinstance(self.data, np.ma.core.MaskedArray) \
            else np.ones_like(self.data)
        self.searchKernel = SearchKernel(self.kernel_size, self.kernel_type)
        self.ROW_COL = Cartesian(self.nw_geo_point, self.resolution, reverse=True)
        self.COL_ROW = Cartesian(self.nw_geo_point, self.resolution)
        self.cached_neighbours = self._cache_neighbours() if self.cached else []

    def _getMeshElement(self, mesh_coordinates):
        if len(self._inBounds(mesh_coordinates))>0:
            #TODO: need to make this a function:
            mesh_coordinates = mesh_coordinates[0]
            geo_coordinates = mesh_coordinates[::-1]*self.resolution
            return MeshElement(self, mesh_coordinates, geo_coordinates)
        else:
            raise IndexError("The location (%s, %s) is out of bounds" %
                             (mesh_coordinates[0], mesh_coordinates[1]))

    def _getNeighbours(self, mesh_coordinates):
        state = np.array(mesh_coordinates)
        kernel = self.searchKernel
        offset = kernel.getKernel()
        if self.cached:
            selection = self.cached_neighbours[state[0],state[1]]
            passable_neighbours = state + offset[selection]
        else:
            potential_neighbours = state + offset
            passable_neighbours = self._isPassable(potential_neighbours)
        # TODO: need to make this a function:
        coordsxy =  np.fliplr(passable_neighbours)*self.resolution
        return MeshCollection(self, passable_neighbours.transpose(), coordsxy.transpose())

    def setSlopes(self):
        [gx, gy] = np.gradient(self.data, self.resolution, self.resolution)
        self.slopes = np.degrees(
            np.arctan(np.sqrt(np.add(np.square(gx), np.square(gy)))))  # Combining for now for less RAM usage

    def setRadialKeepOutZone(self, center, radius):
        circlex, circley = filled_grid_circle(radius)
        self.obstacles[center+circlex, center+circley] = 1

    def getElevations(self, mesh_coordinates):
        #IMPORTANT assumes the elevation requested is not masked
        row, col = mesh_coordinates
        return self.dataset_unmasked[row,col]

    @coordinate_transform
    def getSlope(self, coordinates):
        row, col = coordinates
        return self.slopes[row,col]

    def _inBounds(self, mesh_coordinates):
        # determines if a coordinate is within the boundaries of the environmental model
        boolean = self._inbounds_bool(mesh_coordinates)
        return mesh_coordinates[boolean]

    def _inbounds_bool(self, mesh_coordinates):
        below_bounds = mesh_coordinates < self.shape
        over_bounds = mesh_coordinates >= np.zeros(2)
        boolean = np.logical_and(below_bounds, over_bounds).all(1)
        return boolean

    def _hasdata(self, mesh_coordinates):
        bounded = self._inBounds(mesh_coordinates)
        row, col = bounded.transpose()
        return bounded[self.isvaliddata[row, col]]

    @coordinate_transform
    def elt_hasdata(self, mesh_coordinate):
        return len(self._hasdata(mesh_coordinate)) > 0

    def maxSlopeObstacle(self, maxSlope):
        self.obstacles = self.slopes > maxSlope
        self.passable = np.logical_not(self.obstacles)

    def _isPassable(self, mesh_coordinates):
        valid_data = self._hasdata(mesh_coordinates)
        row, col = valid_data.transpose()
        return valid_data[self.passable[row, col]]

    def convert_coordinates(self, coordinates):
        if isinstance(coordinates, GeoPoint):
            return np.array([coordinates.to(self.ROW_COL)])
        elif isinstance(coordinates, np.ndarray):
            return coordinates
        else:
            return np.array([coordinates])

    #TODO: move to parent class
    def cache_neighbours(self):
        self.cached_neighbours = self._cache_neighbours()
        return self.cached_neighbours

    def _cache_neighbours(self):
        print('cashing neighbours')
        rows, cols = np.mgrid[0:self.y_size, 0:self.x_size]
        gridpoints = np.array([rows.flatten(), cols.flatten()]).transpose()

        kernel = self.searchKernel
        offsets = kernel.getKernel()
        s = np.empty((self.shape[0], self.shape[1], len(offsets)), dtype=bool)
        for idx, offset in enumerate(offsets):
            candidate_neighbour_point = gridpoints + offset
            point_is_neighbour = self._inbounds_bool(candidate_neighbour_point) # if not in bounds, its not a neighbour
            inbound_rows, inbound_cols = candidate_neighbour_point[point_is_neighbour].transpose()
            point_is_neighbour[point_is_neighbour] = self.isvaliddata[inbound_rows, inbound_cols]
            hasdata_rows, hasdata_cols = candidate_neighbour_point[point_is_neighbour].transpose()
            point_is_neighbour[point_is_neighbour] = self.passable[hasdata_rows, hasdata_cols]
            s[:,:,idx] = np.reshape(point_is_neighbour, self.shape)
        return s


def loadElevationMap(fullPath, maxSlope=35, nw_corner=None, se_corner=None, desiredRes=None):
    geoenvelope = GeoEnvelope(nw_corner, se_corner)
    maxSlope=35 #TODO: need to fix this
    dem = GDALMesh(fullPath)
    return dem.loadSubSection(geoenvelope, maxSlope=maxSlope, desired_res=desiredRes)


if __name__ == '__main__':
    from pextant.lib.utils import gridpoints_list
    ames_em = GDALMesh('../data/maps/Ames/Ames.tif').loadSubSection(maxSlope=25)
    #cProfile.run('ames_em.cache_neighbours()')
    s = ames_em.cache_neighbours()
    positions = gridpoints_list(ames_em)
    kernel = ames_em.searchKernel.getKernel()
    for position in positions:
        n1 = ames_em._getNeighbours(position).mesh_coordinates
        n2 = (position + kernel[s[position[0], position[1]]]).transpose()
        if not np.array_equal(n1, n2):
            print 'Houston we have a problem'
    print "Houston we don't have a problem'"
