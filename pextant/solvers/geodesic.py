from pextant.solvers.SEXTANTsolver import SEXTANTSolver
import numpy as np
import gdist
from trimesh import Trimesh
from pextant.mesh.triangularmesh import TriDataset, TriMeshModel, GeoMesh, TriMeshPLY
from pextant.mesh.utils import delaunaytriang
from collections import defaultdict

class gdistSolver(SEXTANTSolver):
    def __init__(self, environmental_model, explorer, viz=None, type='vertex', weight=1):
        super(gdistSolver, self).__init__(environmental_model, explorer, viz)
        self.type = type
        self.get_tmm = dict(vertex=self.get_tmm_vertex, center=self.get_tmm_center)
        self.energy_cost = self.calculate_energy_cost()
        self.weight = weight

    def solve(self, start_point, end_point):
        tmm = self.get_tmm[self.type]()
        #tmm = self.env_model
        mesh = tmm.data
        source = np.array([int(tmm.find_nearest((start_point[1], start_point[0])))])
        target = np.array([int(tmm.find_nearest((end_point[1], end_point[0])))])
        gdist_vertices = mesh.vertices
        gdist_faces = mesh.faces.astype('int32')
        path = gdist.compute_path(gdist_vertices, gdist_faces, source_indices=source, target_indices=target)
        return path

    def get_tmm_center(self):
        z_cost = self.weight*self.energy_cost
        vertices = np.c_[self.env_model.data.triangles_center[:, :2], z_cost]
        mesh = TriMeshPLY.from_poly(delaunaytriang(vertices))
        dataset = TriDataset(mesh)
        tmm = TriMeshModel.from_parent(GeoMesh(self.env_model.nw_geo_point, dataset, self.env_model.resolution,
                                         xoff=self.env_model.xoff, yoff=self.env_model.yoff))
        return tmm

    def get_tmm_vertex(self):
        z_cost = self.weight*self.solve_vertex()
        vertices = np.c_[self.env_model.data.vertices[:, :2], z_cost]
        mesh = Trimesh(vertices, self.env_model.data.faces)
        dataset = TriDataset(mesh)
        tmm = TriMeshModel.from_parent(GeoMesh(self.env_model.nw_geo_point, dataset, self.env_model.resolution,
                                         xoff=self.env_model.xoff, yoff=self.env_model.yoff))
        return tmm

    def solve_vertex(self):
        energy_cost = self.energy_cost
        neighbouring_faces = self.calculate_neighbouring_faces()
        z_cost = [np.mean(energy_cost[neighbouring_face]) for neighbouring_face in neighbouring_faces.itervalues()]
        return np.array(z_cost)

    def calculate_neighbouring_faces(self):
        neighbouring_faces = defaultdict(list)
        for face_idx, vertices in enumerate(self.env_model.data.faces):
            for vertex_idx in vertices:
                neighbouring_faces[vertex_idx].append(face_idx)
        return neighbouring_faces

    def calculate_energy_cost(self):
        # Calculates the slope cost at the center of each triangle
        mesh = self.env_model.data
        nx, ny, nz = tuple(mesh.face_normals[:, :].transpose())
        thmax = np.arctan2(ny, nx)
        steepest_dir = np.array([np.cos(thmax), np.sin(thmax), np.zeros(thmax.size)]).transpose()
        steepest_slope = np.pi / 2 - np.arccos(np.sum(steepest_dir * mesh.face_normals, 1))
        slopes_rad = steepest_slope #face based
        area_face = mesh.area_faces
        average_length = np.ones_like(area_face)
        energy_cost, _ = self.cost_function.energy_expenditure(
            average_length, slopes_rad, 9.81)
        return energy_cost