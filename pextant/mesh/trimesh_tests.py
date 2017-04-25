from pextant.lib.geoshapely import GeoPoint
from pextant.mesh.triangularmesh import TriMesh
from pextant.settings import AMES_DEM

tm = TriMesh(AMES_DEM)
em = tm.loadSubSection()
testpoint = GeoPoint(em.ROW_COL, 20, 30)

faceidx = em.convert_coordinates(testpoint)
print em._getNeighbours(faceidx)
