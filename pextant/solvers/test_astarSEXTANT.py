from astarSEXTANT import ExplorerCost
from pextant.ExplorerModel import Astronaut
from pextant.EnvironmentalModel import GDALMesh
from pextant.settings import AMES_DEM
from pextant.MeshModel import MeshElement

ames_em = GDALMesh(AMES_DEM).loadMapSection()
a = Astronaut(80)
costfunction = ExplorerCost(a, ames_em, 'Energy')
node = ames_em._getNeighbours(10,10)

costs = costfunction.calculateCostBetween(ames_em._getMeshElement(10,10), ames_em._getNeighbours(10,10))
print costs