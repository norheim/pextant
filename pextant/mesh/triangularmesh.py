import numpy as np
from trimesh import load_mesh, proximity
from pextant.mesh.utils import delaunaytriang, decimate, write_poly
from pextant.EnvironmentalModel import GDALMesh, EnvironmentalModel, coordinate_transform
from pextant.lib.geoshapely import GeoPoint, XY
from pextant.mesh.abstractmesh import GeoMesh
from pextant.mesh.abstractcomponents import MeshElement, MeshCollection


class TriDataset(object):
    def __init__(self, mesh, elevations=None, y_size=None, x_size=None):
        self.elevations = elevations
        if x_size == None:
            x_size, y_size, _ = np.diff(mesh.mesh.bounds.transpose()).flatten().astype('int32')
        self.shape = (y_size, x_size)
        self.size,_ = mesh.faces.shape
        self.mesh = mesh
        self.mask = np.zeros(self.size) # for missing data


class TriMeshPLY(object):
    def __init__(self, file_path):
        self.file_path = file_path
        self.mesh = load_mesh(file_path)

    # wrapper around trimesh for easy debugging and interacting with the mesh
    def saveas(self, filename):
        self.mesh.export(filename)

    @classmethod
    def from_poly(cls, poly):
        temp_filename = 'temporary.ply'
        write_poly(poly, temp_filename)
        return cls(temp_filename)

    def __getattr__(self, item):
        try:
            return TriMeshPLY.__getattribute__(self, item)
        except AttributeError:
            return getattr(self.mesh, item)

class GDALTriMesh(GDALMesh):
    def __init__(self, file_path):
        super(GDALTriMesh, self).__init__(file_path)

    #TODO: cache elevations
    def _loadSubSectionTri(self, geo_envelope=None, desired_res=None, accuracy=0.05):
        sub_mesh = self._loadSubSection(geo_envelope, desired_res)
        row_size, col_size = sub_mesh.shape
        magnification = 1 / self.resolution

        # clean data set
        subsection_elevations = sub_mesh.dataset
        min_elevation = np.min(subsection_elevations)
        subsection_elevations_corrected = subsection_elevations.filled(min_elevation)
        grounded_elevations = subsection_elevations_corrected - min_elevation
        stretch_elevations = magnification * grounded_elevations  # force z resolution to match x & y

        corrected_absolute_error = magnification * accuracy
        poly = decimate(stretch_elevations, corrected_absolute_error)
        mesh = TriMeshPLY.from_poly(poly).mesh
        dataset = TriDataset(mesh, stretch_elevations, row_size, col_size)

        return GeoMesh(sub_mesh.nw_geo_point, dataset, sub_mesh.resolution, parent_mesh=sub_mesh.parent_mesh,
                       xoff=sub_mesh.x_offset, yoff=sub_mesh.y_offset)

    def loadSubSection(self, geo_envelope=None, desired_res=None, accuracy=0.05, **kwargs):
        sub_mesh = self._loadSubSectionTri(geo_envelope, desired_res, accuracy)
        return TriMeshModel.from_parent(sub_mesh, **kwargs)


class TriMeshModel(EnvironmentalModel):
    def __init__(self, *arg, **kwargs):
        super(TriMeshModel, self).__init__(*arg, **kwargs)
        self.raster = self.dataset.elevations
        self.faces = self.dataset.mesh.faces
        self.neighbours = dict()
        #self.xy = XY(self.nw_geo_point, 1)
        self.ROW_COL = XY(self.nw_geo_point, self.resolution, reverse=True)
        self.COL_ROW = XY(self.nw_geo_point, self.resolution)
        self.cache_neighbours()

    def cache_neighbours(self):
        self.neighbours = self.edge_neighbours()

    def all_neighbours(self):
        d = dict()
        for idx, mesh_face in enumerate(self.faces):
            for vertex in range(3):
                d.setdefault(mesh_face[vertex], []).append(idx)
        return d

    def edge_neighbours(self):
        d = dict()
        for adjacent in self.dataset.mesh.face_adjacency:
            d.setdefault(adjacent[0], []).append(adjacent[1])
            d.setdefault(adjacent[1], []).append(adjacent[0])
        return d

    def _getMeshElement(self, faceidx):
        # TODO: need to make this a function:
        x, y, _  = self.dataset.mesh.triangles_center[faceidx]*self.resolution
        return MeshElement(self, faceidx, (x, y))

    def _getNeighbours(self, faceidx):
        #potential_neighbours = reduce(np.union1d, map(self.neighbours.get, self.faces[faceidx]))
        #potential_neighbours = np.setdiff1d(potential_neighbours, np.array(faceidx))
        potential_neighbours = self.neighbours[faceidx]
        passable_neighbours = np.array(filter(self._isPassable, potential_neighbours))
        triangles = self.dataset.mesh.triangles_center[passable_neighbours]
        # TODO: need to make this a function:
        xycoords = triangles[:,:2].transpose()*self.resolution
        return MeshCollection(self, passable_neighbours, xycoords)

    def setSlopes(self):
        nx, ny, nz =  tuple(self.dataset.mesh.face_normals[:, :].transpose())
        thmax = np.arctan2(ny, nx)
        steepest_dir = np.array([np.cos(thmax), np.sin(thmax), np.zeros(thmax.size)]).transpose()
        steepest_slope = 90 - np.degrees(np.arccos(np.sum(steepest_dir * self.dataset.mesh.face_normals, 1)))
        self.slopes = steepest_slope
        return steepest_slope

    def setRadialKeepOutZone(self, center, radius):
        pass

    def getElevations(self, mesh_coordinates):
        #faceidx can be one or many
        _,_,z = self.dataset.mesh.triangles_center[mesh_coordinates].transpose()
        return z*self.resolution

    def _inBounds(self, coordinates):
        return coordinates

    def _hasdata(self, coordinates):
        return self.isvaliddata[coordinates]

    def convert_coordinates(self, coordinates):
        if isinstance(coordinates, GeoPoint):
            x, y = coordinates.to(self.COL_ROW)
            z = self.raster[int(np.round(y)),int(np.round(x))]/self.resolution
            _,_, facidx = proximity.closest_point(self.dataset.mesh, [[x, y, z]])
            return facidx[0]
        else:
            return coordinates