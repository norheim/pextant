import docker
from pextant.maps.generate_colormap import colormap
import numpy as np
from pextant.EnvironmentalModel import loadElevationsLite, selectMapSection

client = docker.from_env()

def colorRelief(file_path, geo_envelope=None):
    dataset, all_info = loadElevationsLite(file_path)
    if geo_envelope is not None:
        _, dataset = selectMapSection(dataset, all_info, geo_envelope)

    altitude_lower_bound, altitude_upper_bound = np.min(dataset), np.max(dataset)

    N = 100 # levels of colors
    colormap(N, altitude_lower_bound, altitude_upper_bound)
    # generates colormap.txt

    command = "gdaldem color-relief %s colormap.txt %s_relief_%s.tif -nearest_color_entry" \
              % (file_path, file_path, N)
    container = "spara/gdal_ef"
    volume = {
        '/c/Users/johan/Dropbox (MIT)/BASALT/pextant/pextant/maps':
            {'bind': '/data',
             'mode': 'rw'},
        }
    client.containers.run(container, command, volumes=volume)