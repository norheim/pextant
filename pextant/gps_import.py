from api import *
import matplotlib.pyplot as plt
from osgeo import gdal
import pandas as pd
import numpy as np
pd.options.display.max_rows = 5

map = loadElevationMap('hsnew.tif', maxSlope = 15,
					planet = 'Earth', NWCorner = None, SECorner = None, desiredRes = 0.5)

def get_gps_data(filename, traversal_id):
    """
    Gets GPS time series gathered from a traversal
    :param filename: <String> csv file from GPS team in format |date|time|name|latitude|longitude|heading
    :param traversal_id: <String> name of traversal we would like further information on
    :return: <pandas DataFrame> time_stamp|latitude|longitude
    """
    delimiter = r"\s+" # some of the columns are separated by a space, others by tabs, use regex to include both
    header_row = 0     # the first row has all the header names
    df = pd.read_csv(filename, sep=delimiter, header=header_row,
                     parse_dates=[['date', 'time']])    # replace date and time columns with date_time variable
    time_lat_long = df[df['name'] == traversal_id][['date_time', 'latitude', 'longitude']]
    return time_lat_long


#TODO: Need to move this over to test file
filename = 'cotm_16_tracks.csv'
traversal_id = '20160629A_EV1'

time_lat_long = get_gps_data(filename, traversal_id)
gps_row_col = []
for index, row in time_lat_long.iterrows():
    gps_row_col = gps_row_col.append([row.latitude, row.longitude])

gps_row_co_np = np.array(gps_row_col)
gps_coordinates = LatLongCoord(gps_row_co_np[:,0], gps_row_co_np[:,1])

#ap1 = ActivityPoint((179, 149), 0)
#ap2 = ActivityPoint((42.0, 71.0), 0)

#final = P.aStarCompletePath([0, 0, 1], [ap1, ap2], 'tuple')
#print final
#path_x = [point[0] for point in final[0]]
#path_y = [point[1] for point in final[0]]

ds = gdal.Open('hsnew.tif')
band = ds.GetRasterBand(1)
arr = band.ReadAsArray()

#plt.plot([149,71], [179,42], 'ro')
plt.imshow(arr, cmap = 'viridis')
#plt.plot(path_y, path_x)
plt.plot(gps_row_col)
plt.show()
