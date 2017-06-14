from vtk import vtkImageData, vtkGreedyTerrainDecimation, VTK_FLOAT, vtkPolyData, vtkPoints, vtkPLYWriter, \
    vtkDelaunay2D, vtkRenderWindow, vtkWindowToImageFilter, vtkPNGWriter, vtkActor, vtkRenderer, vtkPolyDataMapper
from vtk.util.numpy_support import numpy_to_vtk
from IPython.display import Image

def write_poly(polygons, filename):
    plyWriter = vtkPLYWriter()
    plyWriter.SetFileName(filename)
    plyWriter.SetInputConnection(polygons.GetOutputPort())
    plyWriter.Write()

def delaunaytriang(points_xyz):
    vtk_points = vtkPoints()
    for point_xyz in points_xyz:
        x, y, z = point_xyz
        vtk_points.InsertNextPoint(x, y, z)

    inputPolyData = vtkPolyData()
    inputPolyData.SetPoints(vtk_points)

    delny = vtkDelaunay2D()
    delny.SetInputData(inputPolyData)
    delny.Update()

    return delny

def decimate(zarray, corrected_absolute_error):
    row_size, col_size = zarray.shape
    number_of_elevation_entries = zarray.size
    vectorized_elevations = np.reshape(zarray, (number_of_elevation_entries, 1))
    vtk_array = numpy_to_vtk(vectorized_elevations, deep=True,
                             array_type=VTK_FLOAT)
    image = vtkImageData()
    image.SetDimensions(col_size, row_size, 1)
    image.AllocateScalars(vtk_array.GetDataType(), 4)
    image.GetPointData().GetScalars().DeepCopy(vtk_array)

    deci = vtkGreedyTerrainDecimation()
    deci.SetInputData(image)
    deci.BoundaryVertexDeletionOn()
    deci.SetErrorMeasureToAbsoluteError()
    deci.SetAbsoluteError(corrected_absolute_error)
    deci.Update()

    return deci

def get_number_of_polys(output):
        decimated_poly = vtkPolyData()
        decimated_poly.ShallowCopy(output.GetOutput())
        number_of_polys = decimated_poly.GetNumberOfPolys()
        return number_of_polys

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
