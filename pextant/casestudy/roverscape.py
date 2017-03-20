from pextant.api import Pathfinder
from pextant.EnvironmentalModel import GDALMesh
from pextant.ExplorerModel import Astronaut
from pextant.lib.geoshapely import GeoEnvelope, GeoPoint, LAT_LONG
import matplotlib.pyplot as plt
from pextant.solvers.astarSEXTANT import ExpandViz, ExplorerCost, fullSearch
import numpy as np
import cProfile

def roverscape():
    dem_map = GDALMesh('../../data/maps/Ames/Ames.tif')
    env_model = dem_map.loadMapSection()
    #viz = ExpandViz(env_model)
    #viz.draw()
    astronaut = Astronaut(70)
    #P = Pathfinder(astronaut, env_model)

    waypoint1 = GeoPoint(env_model.ROW_COL,2,2)
    waypoint2 = GeoPoint(env_model.ROW_COL,40,120)
    cost_function = ExplorerCost(astronaut, env_model, 'Energy')
    segmentsout, rawpoints, items = fullSearch([waypoint1, waypoint2], env_model, cost_function)
    #segmentsout, rawpoints, itemssearched = P.completeSearch('Energy', [waypoint1, waypoint2])
    #print len(itemssearched)
    #viz.drawsolution(rawpoints)
    #print segmentsout

if __name__ == '__main__':
    #roverscape()
    cProfile.run('roverscape()')
