import numpy as np
import numpy.matlib as matlib
import json
from pextant.lib.geoshapely import GeoPoint, LAT_LONG

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
        self.length = kernelrange**2
        self.col_off = col_off.flatten()
        self.row_off = row_off.flatten()

    def getKernel(self):
        return np.array([self.col_off, self.row_off])

class MeshElement(object):
    def __init__(self, row, col, parentMesh):
        self.row = row
        self.col = col
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
        return np.array([self.row, self.col])

    def getTupleCoords(self):
        return (self.row, self.col)

    def getElevation(self):
        return self.parentMesh.dataset[self.row, self.col]

    def getNeighbours(self):
        return self.parentMesh.getNeighbours(self)

    def __str__(self):
        return '(%s, %s)' % (self.row, self.col)

class MeshCollection(object):
    def __init__(self):
        self.collection = []
    def raw(self):
        return [(mesh_elt.row, mesh_elt.col) for mesh_elt in self.collection]
    def __str__(self):
        return str(self.raw())