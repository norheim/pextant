import numpy as np
from pykml import parser
from geoshapely import GeoPolygon, LONG_LAT
import re

def gridpoints_list(array):
    X, Y = np.mgrid[0:array.shape[0], 0:array.shape[1]]
    positions = np.column_stack((X.flatten(), Y.flatten()))
    return positions

def kml2shapely(kmlfile):
    with open(kmlfile) as f:
        doc = parser.parse(f)
    root = doc.getroot()
    coords = root.Document.Placemark.Polygon.outerBoundaryIs.LinearRing.coordinates
    str1 = (str(coords)).split(' ')[0]
    allcoords = re.findall('-?\d+\.*\d*', str(coords))
    coordarray = np.array(allcoords, dtype=float).reshape(len(allcoords)/3, 3)[:,:2]
    ring = GeoPolygon(LONG_LAT, *coordarray.transpose())
    return ring