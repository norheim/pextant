import numpy as np
import json

class MeshElement(object):
    def __init__(self, parentMesh, state, representative_point):
        if isinstance(state, np.ndarray):
            state = tuple(state)
        self.state = state
        self.x, self.y = representative_point
        self.z = parentMesh.getElevations(state)
        self.timedelta = 0
        self.resolution = parentMesh.resolution
        self.parentMesh = parentMesh

    def getBorders(self):
        pass

    def getState(self):
        return self.state

    def getNeighbours(self):
        return self.parentMesh._getNeighbours(self.state)

    #TODO: need to add third dimension (other_elt.getElevation() - self.getElevation())**2
    def distanceTo(self, other_elts):
        """

        :param other_elts:
        :type other_elts: MeshCollection
        :return:
        """
        path_length = np.sqrt(
            (self.x - other_elts.x) ** 2 +
            (self.y - other_elts.y) ** 2)
        return path_length

    def distanceToElt(self, other_elt):
        """

        :param other_elt:
        :type other_elt: MeshElement
        :return:
        """
        if isinstance(other_elt.state, int): #TODO: make custom element
            path_length = (other_elt.state != self.state)*self.parentMesh.resolution*3
        else:
            path_length = np.sqrt(
                (self.x - other_elt.x) ** 2 +
                (self.y - other_elt.y) ** 2)
        return path_length

    def slopeTo(self, other_elts):
        """

        :param other_elts:
        :type other_elts: MeshCollection
        :return:
        """
        path_length = self.distanceTo(other_elts)
        #TODO: how to avoid divide by zero?
        slopes = np.degrees(np.arctan((other_elts.z - self.z) / path_length))
        return slopes, path_length

    def __str__(self):
        return '(%s, %s, %s, %s)' % (self.state, self.x, self.y, self.z)

    def settime(self, time):
        self.timedelta = time

class MeshCollection(object):
    def __init__(self, parentmesh, mesh_coordinates, geo_coordinates):
        self.parentmesh = parentmesh
        self.mesh_coordinates = mesh_coordinates
        self.coordinates = geo_coordinates
        self.x, self.y = geo_coordinates if geo_coordinates.size else np.array([[], []])
        self.z = parentmesh.getElevations(mesh_coordinates)

    def __getitem__(self, index): #TODO: clean this up
        if self.mesh_coordinates.ndim ==1:
            mesh_coordinates = self.mesh_coordinates[index]
        else:
            mesh_coordinates = self.mesh_coordinates[:,index]
        geo_point = self.coordinates[:,index]
        return MeshElement(self.parentmesh, mesh_coordinates, geo_point)

    def get_states(self):
        if self.mesh_coordinates.ndim == 1:
            return self.mesh_coordinates
        else:
            return map(tuple, self.mesh_coordinates.transpose())

    def raw(self):
        return np.array(self.coordinates)

    def distances(self):
        pass

    def velocities(self):
        pass

    def __str__(self):
        return str(self.raw())

if __name__ == '__main__':
    from pextant.EnvironmentalModel import GDALMesh
    from pextant.mesh.triangularmesh import TriMesh
    from pextant.lib.geoshapely import GeoPoint
    ames_em = TriMesh('../../data/maps/Ames/Ames.tif').loadSubSection()
    #out = ames_em._getNeighbours((10,20))
    out = ames_em._getNeighbours(2625)
    otherelt = ames_em._getMeshElement(10)
    #print(out.__getitem__(2).distanceToElt(ames_em._getMeshElement(10)))
    print out
    print(out.mesh_coordinates)

