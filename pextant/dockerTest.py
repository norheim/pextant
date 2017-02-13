import docker
import numpy as np
from loadWaypoints import loadPoints
from geoshapely import LAT_LONG
from EnvironmentalModel import loadElevationsLite, selectMapSection
from generate_colormap import colormap
import shutil

client = docker.from_env()

def colorRelief(output_dir, folder, file_name, N, geo_envelope=None):
    file_extension = '%s.tif' % file_name
    file_path = '%s\\%s' % (folder, file_extension)
    dataset, all_info = loadElevationsLite(file_path)
    _, dataset, window = selectMapSection(dataset, all_info, geo_envelope)

    clean_dataset = dataset[dataset>0]
    altitude_lower_bound, altitude_upper_bound = np.min(clean_dataset), np.max(clean_dataset)

    colormap(N, altitude_lower_bound, altitude_upper_bound)
    # generates colormap.txt
    zoom_name = "%s_zoomed.tif" % file_name
    zoom_command = 'gdal_translate %s %s -srcwin %s %s %s %s' \
                   % (file_extension, zoom_name, window['xoff'], window['yoff'], window['xsize'], window['ysize'])
    relief_name = "%s_relief_%s" % (file_name, N)
    relief_extension  = "%s.tif" % relief_name
    command = "gdaldem color-relief %s colormap.txt %s -nearest_color_entry" \
              % (zoom_name, relief_extension)
    tile_command = "gdal2tiles.py %s" % relief_extension

    docker_cmd = 'bash -c "%s" && "%s"' % (zoom_command, command)
    container = "spara/gdal_ef"
    volume = {
        '/c/Users/johan/Dropbox (MIT)/BASALT/pextant/pextant/maps':
            {'bind': '/data',
             'mode': 'rw'},
        }
    client.containers.run(container, zoom_command, volumes=volume)
    client.containers.run(container, command, volumes=volume)
    client.containers.run(container, tile_command, volumes=volume)

    shutil.move("C:\Users\johan\Dropbox (MIT)\BASALT\pextant\pextant\maps\\"+relief_name,
                output_dir + relief_name)

if __name__ == '__main__':
    output_dir = 'C:\Users\johan\Desktop\webapp\public\CustomMaps\\'
    folder = 'C:\Users\johan\Dropbox (MIT)\BASALT\pextant\pextant\maps\\'
    file_name = 'HI_air_imagery'
    points = loadPoints('waypoints/HI_13Nov16_MD7_A.json')
    geoEnvelope = points.geoEnvelope()

    colorRelief(output_dir, folder, file_name, 100, geoEnvelope)
