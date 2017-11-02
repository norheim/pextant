from folium import Map, GeoJson, PolyLine, LayerControl, TileLayer
from pextant.lib.geoshapely import LAT_LONG

class GeoViz(object):
    def __init__(self, map):
        self.map = map

    def drawpath(self, poly_points):
        poly = list(poly_points.to(LAT_LONG).transpose())
        polyf = PolyLine(locations=poly,popup='MD%').add_to(self.map)