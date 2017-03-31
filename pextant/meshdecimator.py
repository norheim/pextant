#!/usr/bin/env python

# This example shows how to use Delaunay3D with alpha shapes.

import vtk

# The points to be triangulated are generated randomly in the unit
# cube located at the origin. The points are then associated with a
# vtkPolyData.
math = vtk.vtkMath()
points = vtk.vtkPoints()
for i in range(0, 100000):
    points.InsertPoint(i, math.Random(0, 1), math.Random(0, 1),
                       math.Random(0, 1))

profile = vtk.vtkPolyData()
profile.SetPoints(points)

print("Before decimation\n"
          "-----------------\n"
          "There are " + str(profile.GetNumberOfPoints()) + "points.\n"
          "There are " + str(profile.GetNumberOfPolys()) + "polygons.\n")

# Delaunay3D is used to triangulate the points. The Tolerance is the
# distance that nearly coincident points are merged
# together. (Delaunay does better if points are well spaced.) The
# alpha value is the radius of circumcircles, circumspheres. Any mesh
# entity whose circumcircle is smaller than this value is output.
delny = vtk.vtkDelaunay3D()
delny.SetInputData(profile)
delny.SetTolerance(0.01)
delny.SetAlpha(0.5)
delny.BoundingTriangulationOff()

mapMesh = vtk.vtkPolyDataMapper()
mapMesh.SetInputConnection(delny.GetOutputPort())

decimate = vtk.vtkDecimatePro()
decimate.SetInputData(profile)
decimate.SetTargetReduction(.10)
decimate.Update()

decimatedPoly = vtk.vtkPolyData()
decimatedPoly.ShallowCopy(decimate.GetOutput())
print("After decimation \n"
          "-----------------\n"
          "There are " + str(decimatedPoly.GetNumberOfPoints()) + "points.\n"
          "There are " + str(decimatedPoly.GetNumberOfPolys()) + "polygons.\n")