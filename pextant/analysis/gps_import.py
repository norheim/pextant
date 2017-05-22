from pextant.lib.geoshapely import GeoPolygon, LAT_LONG
import matplotlib.pyplot as plt
from osgeo import gdal
import pandas as pd
import numpy as np
pd.options.display.max_rows = 5


def get_gps_data(filename, traversal_id):
    """
    Gets GPS time series gathered from a traversal
    :param filename: <String> csv file from GPS team in format |date|time|name|latitude|longitude|heading
    :return: <pandas DataFrame> time_stamp|latitude|longitude
    """
    delimiter = r"\s+" # some of the columns are separated by a space, others by tabs, use regex to include both
    header_row = 0     # the first row has all the header names
    df = pd.read_csv(filename, sep=delimiter, header=header_row,
                     parse_dates=[['date', 'time']])    # replace date and time columns with date_time variable
    time_lat_long = df[df['name'] == traversal_id][['date_time', 'latitude', 'longitude']]
    gp = GeoPolygon(LAT_LONG, *time_lat_long[['latitude', 'longitude']].as_matrix().transpose())
    return gp

if __name__ == '__main__':
    gps = get_gps_data('../../data/waypoints/cotm_16_tracks.csv', '20160629A_EV1')