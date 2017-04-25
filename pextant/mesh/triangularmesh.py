import numpy as np
from IPython.display import Image
from trimesh import load_mesh, proximity
from vtk import vtkImageData, vtkGreedyTerrainDecimation, VTK_FLOAT, vtkPolyData, vtkPoints, vtkPLYWriter, \
    vtkDelaunay2D, vtkRenderWindow, vtkWindowToImageFilter, vtkPNGWriter, vtkActor, vtkRenderer, vtkPolyDataMapper
from vtk.util.numpy_support import numpy_to_vtk

from pextant.EnvironmentalModel import GDALMesh, EnvironmentalModel, coordinate_transform
from pextant.lib.geoshapely import GeoPoint, XY
from pextant.mesh.abstractcomponents import MeshElement, MeshCollection


class TriDataset(object):
    def __init__(self, elevations, mesh, y_size, x_size):
        self.elevations = elevations
        self.shape = (y_size, x_size)
        self.size,_ = mesh.faces.shape
        self.mesh = mesh
        self.mask = np.zeros(self.size) # for missing data

class TriMesh(GDALMesh):
    def __init__(self, file_path):
        super(TriMesh, self).__init__(file_path)
        self.name = "test.ply"

    def preloadSubSection(self, geo_envelope=None, desired_res=None, **kwargs):
        #kwargs is reserved for max slope argument
        (subsection_elevations, desired_res, nw_coord, x_offset, y_offset) = self._loadSubSection(geo_envelope, desired_res)
        row_size, col_size = subsection_elevations.shape
        subsection_elevations_corrected = subsection_elevations.filled(np.min(subsection_elevations))


        self.section_elevations = subsection_elevations_corrected
        self.section_resolution = desired_res
        self.section_row_size, self.section_col_size = row_size, col_size
        return (subsection_elevations, desired_res, nw_coord, x_offset, y_offset)

    def loadSubSection(self, geo_envelope=None, desired_res=None, accuracy=0.05, **kwargs):
        (subsection_elevations, desired_res, nw_coord, x_offset, y_offset) = \
            self.preloadSubSection(geo_envelope, desired_res, **kwargs)
        clean_elevations = self.clean_dataset()
        self.decimate(clean_elevations, accuracy)
        mesh = self._load(self.name)
        y_size, x_size = subsection_elevations.shape
        dataset = TriDataset(clean_elevations, mesh, y_size, x_size)
        return TriMeshModel(nw_coord, dataset, desired_res,
                            parent_mesh=self, xoff=x_offset, yoff=y_offset, **kwargs)

    def clean_dataset(self):
        #flipped_subsection = np.flipud(self.section_elevations)  # mirror accross x axis
        flipped_subsection = self.section_elevations
        grounded_elevations = flipped_subsection - np.min(flipped_subsection)
        stretch_elevations = 1 / self.resolution * grounded_elevations  # force z resolution to match x & y
        return stretch_elevations

    def decimate(self, stretch_elevations, absolute_error_desired):
        number_of_elevation_entries = self.section_col_size * self.section_row_size
        vectorized_elevations = np.reshape(stretch_elevations, (number_of_elevation_entries, 1))
        vtk_array = numpy_to_vtk(vectorized_elevations, deep=True,
                                 array_type=VTK_FLOAT)
        image = vtkImageData()
        image.SetDimensions(self.section_col_size, self.section_row_size, 1)
        image.AllocateScalars(vtk_array.GetDataType(), 4)
        image.GetPointData().GetScalars().DeepCopy(vtk_array)

        corrected_absolute_error = absolute_error_desired/self.resolution
        deci = vtkGreedyTerrainDecimation()
        deci.SetInputData(image)
        deci.BoundaryVertexDeletionOn()
        deci.SetErrorMeasureToAbsoluteError()
        deci.SetAbsoluteError(corrected_absolute_error)
        deci.Update()
        self._save(deci)
        print(self._get_number_of_polys(deci))
        return deci

    def delaunaytriang(self):
        rows, cols = np.meshgrid(np.arange(self.section_row_size), np.arange(self.section_col_size))
        npp = np.array([rows.flatten(), cols.flatten()]).transpose()
        points = vtkPoints()
        for p in npp:
            row, col = p
            # x=col, y=row, and need to invert rows
            x, y, z = col, self.section_row_size - 1 - row, self.section_elevations[row, col]
            points.InsertNextPoint(x, y, z)

        inputPolyData = vtkPolyData()
        inputPolyData.SetPoints(points)
        delny = vtkDelaunay2D()
        delny.SetInputData(inputPolyData)
        delny.Update()

        self._save(delny)
        print(self._get_number_of_polys(delny))
        return delny

    @staticmethod
    def _get_number_of_polys(output):
        decimated_poly = vtkPolyData()
        decimated_poly.ShallowCopy(output.GetOutput())
        number_of_polys = decimated_poly.GetNumberOfPolys()
        return number_of_polys

    def _save(self, polygons):
        filename = self.name
        plyWriter = vtkPLYWriter()
        plyWriter.SetFileName(filename)
        plyWriter.SetInputConnection(polygons.GetOutputPort())
        plyWriter.Write()

    @staticmethod
    def _load(filepath):
        return load_mesh(filepath)

    @staticmethod
    def notebookviz(output):
        width,height = 400, 300
        demMapper = vtkPolyDataMapper()
        demMapper.SetInputConnection(output.GetOutputPort())

        surfaceActor = vtkActor()
        surfaceActor.SetMapper(demMapper)
        surfaceActor.GetProperty().SetDiffuseColor(1.0000, 0.3882, 0.2784)
        surfaceActor.GetProperty().SetSpecularColor(1, 1, 1)
        surfaceActor.GetProperty().SetSpecular(.4)
        surfaceActor.GetProperty().SetSpecularPower(50)

        VtkRenderer = vtkRenderer()
        VtkRenderer.SetBackground(1.0, 1.0, 1.0)
        VtkRenderer.AddActor(surfaceActor)

        renderWindow = vtkRenderWindow()
        renderWindow.SetOffScreenRendering(1)
        renderWindow.AddRenderer(VtkRenderer)
        renderWindow.SetSize(width, height)
        renderWindow.Render()

        windowToImageFilter = vtkWindowToImageFilter()
        windowToImageFilter.SetInput(renderWindow)
        windowToImageFilter.Update()

        writer = vtkPNGWriter()
        writer.SetWriteToMemory(1)
        writer.SetInputConnection(windowToImageFilter.GetOutputPort())
        writer.Write()
        data = str(buffer(writer.GetResult()))

        return Image(data)

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

    def setRadialKeepOutZone(self, center, radius):
        pass

    def getElevations(self, mesh_coordinates):
        #faceidx can be one or many
        _,_,z = self.dataset.mesh.triangles_center[mesh_coordinates].transpose()
        return z*self.resolution

    def _inBounds(self, coordinates):
        return coordinates

    def _hasdata(self, coordinates):
        return not self.ismissingdata[coordinates]

    def convert_coordinates(self, coordinates):
        if isinstance(coordinates, GeoPoint):
            x, y = coordinates.to(self.COL_ROW)
            z = self.raster[int(np.round(y)),int(np.round(x))]/self.resolution
            _,_, facidx = proximity.closest_point(self.dataset.mesh, [[x, y, z]])
            return facidx[0]
        else:
            return coordinates