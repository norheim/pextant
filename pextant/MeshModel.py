import numpy as np
import numpy.matlib as matlib
import json
from pextant.lib.geoshapely import GeoPoint, LAT_LONG

class Mesh(object):
    def __init__(self, nw_geo_point, dataset, resolution=1, planet='Earth',
                 parentMesh=None, xoff=0, yoff=0, width=0, height=0):
        if not (width and height):
            width, height = dataset.shape
        self.nw_geo_point = nw_geo_point
        self.width = width
        self.height = height
        self.resolution = resolution
        self.parentMesh = parentMesh
        self.xoff = xoff
        self.yoff = yoff
        self.dataset = dataset
        self.planet = planet

    def jsonify(self):
        return json.dumps(self.__dict__)

    def __str__(self):
        return 'height: %s \nwidth: %s \nresolution: %s \nnw corner: %s' % \
              (self.height, self.width, self.resolution, str(self.nw_geo_point))

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
        self.kernel = np.array([row_off_clean, col_off_clean]).transpose()

    def getKernel(self):
        return self.kernel

class MeshElement(object):
    def __init__(self, row, col, parentMesh):
        self.row = row
        self.col = col
        self.timedelta = 0
        self.elt_width = parentMesh.resolution
        self.parentMesh = parentMesh

    def getBorders(self):
        nw_corner = GeoPoint(self.parentMesh.ROW_COL, self.row, self.col)
        se_corner = GeoPoint(self.parentMesh.ROW_COL, self.row+1, self.col+1)
        nw_lat, nw_long = nw_corner.to(LAT_LONG)
        se_lat, se_long = se_corner.to(LAT_LONG)

        border = {
            'nw_lat': nw_lat,
            'nw_long:':nw_long,
            'se_lat':se_lat,
            'se_long':se_long
        }
        borderjson = json.dumps(border)
        return borderjson

    def getCoordinates(self):
        return self.row, self.col

    #TODO: remove
    def getTupleCoords(self):
        return (self.row, self.col)

    def getElevation(self):
        return self.parentMesh.dataset[self.row, self.col]

    def getNeighbours(self):
        return self.parentMesh.getNeighbours(self)

    #TODO: need to add third dimension (other_elt.getElevation() - self.getElevation())**2
    def distanceTo(self, other_elts):
        """

        :param other_elts:
        :type other_elts: MeshCollection
        :return:
        """
        path_length = self.elt_width * np.sqrt(
            (self.row - other_elts.getrows()) ** 2 +
            (self.col - other_elts.getcols()) ** 2)
        return path_length

    def slopeTo(self, other_elts):
        """

        :param other_elts:
        :type other_elts: MeshCollection
        :return:
        """
        path_length = self.distanceTo(other_elts)
        #TODO: how to avoid divide by zero?
        slopes = np.degrees(np.arctan((other_elts.get_elevations() - self.getElevation()) / path_length))
        return slopes, path_length

    def __str__(self):
        return '(%s, %s)' % (self.row, self.col)

    def settime(self, time):
        self.timedelta = time


class MeshCollection(object):
    def __init__(self, parentmesh, collection):
        self.parentmesh = parentmesh
        self.collection = collection
        self.rows, self.cols = collection.transpose()

    def add_element(self, elt):
        row, col = elt
        self.rows.append(row)
        self.cols.append(col)
        self.collection.append((row, col))

    def __getitem__(self, index):
        return MeshElement(self.rows[index],self.cols[index],self.parentmesh)

    def getrows(self):
        return self.rows

    def getcols(self):
        return self.cols

    def get_elevations(self):
        return self.parentmesh.dataset[self.rows, self.cols]

    def raw(self):
        return np.array(self.collection)

    def distances(self):
        pass

    def velocities(self):
        pass

    def __str__(self):
        return str(self.raw())

