import os
import numpy as np
from pextant.EnvironmentalModel import GDALMesh
DATA_ROOT = os.path.join('..','..','data')
MAPS_ROOT = os.path.join(DATA_ROOT,'maps')
WAYPOINTS_ROOT = os.path.join(DATA_ROOT,'waypoints')
AMES_DEM = os.path.join(MAPS_ROOT, 'Ames', 'Ames.tif')
ID_DEM_PATH = os.path.join(MAPS_ROOT, 'Idaho', 'hwmidres.tif')
HI_DEM_PATH = os.path.join(MAPS_ROOT,'test5-DEM.tif')
HI_DEM_LOWQUAL_PATH = os.path.join(MAPS_ROOT,'HI_lowqual_DEM.tif')
HI_DEM_AERIAL_PATH = os.path.join(MAPS_ROOT,'dem','HI_air_imagery.tif')
HI_DEM = GDALMesh(HI_DEM_PATH)
HI_DEM_LOWQUAL = GDALMesh(HI_DEM_LOWQUAL_PATH)
HI_DEM_AERIAL = GDALMesh(HI_DEM_AERIAL_PATH)
ID_DEM = GDALMesh(ID_DEM_PATH)
TEST_JSON = os.path.join(WAYPOINTS_ROOT, 'HI_13Nov16_MD7_A.json')
MD_HI = dict()
for i in range(2,10):
    MD_HI[i] = os.path.join(WAYPOINTS_ROOT, ('HI_%02dNov16_MD%s_A.json')%(i+6,i))
ID_MD = {10: os.path.join(WAYPOINTS_ROOT, 'MD10_EVA10_Stn18_Stn23_X.json')}
