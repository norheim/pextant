from pextant.mesh.triangularmesh import TriPolyMesh
from pextant.solvers.astarMesh import MeshSearchElement
ames_em = TriPolyMesh('../../data/maps/Ames/Ames.tif').loadSubSection()
start = MeshSearchElement(ames_em._getMeshElement(10))
#end = MeshSearchElement(ames_em._getMeshElement(10))