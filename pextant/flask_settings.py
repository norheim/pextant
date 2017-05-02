import os

GEOTIFF_PATH = '../../data/dem/'

AMES = 'Ames.tif'
COTM_HIGHWAY = 'COTM_Highway.tif'
COTM_NORTH_CRATER = 'COTM_North_Crater.tif'
HAWAII_MAUNA_ULU = 'Hawaii_Lava_Flows.tif'
HAWAII_MAUNA_KEA = 'TODO_CREATE.tif'

# if no path is passed in as a parameter, this will be used.
GEOTIFF_FULL_PATH = os.path.join(GEOTIFF_PATH, AMES) 