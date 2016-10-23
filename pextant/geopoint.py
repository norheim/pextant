import pyproj
import numpy as np


class GeoType(object):
    def __init__(self, name, values, projparam, projtransformorder):
        self.name = name
        self.projtype = name
        self.values = values
        self.projparam = projparam
        self.proj_transform_order = [values.index(parameter) for parameter in projtransformorder]

    def get_proj(self, data):
        for k, v in self.projparam.iteritems():
            for val in self.values:
                self.projparam[k] = data[v] if v == val else v

        # add additional parameters here
        self.projparam["datum"] = "WGS84"

        proj_param = self.projparam
        p = pyproj.Proj(**proj_param)
        return p

    # will belong to all instance
    def transform(self, geo_point, to_geo_type):
        if self.name == to_geo_type.name:
            return geo_point.values()
        elif self.projtype == to_geo_type.projtype:
            args = self.getargs(geo_point)
            return to_geo_type.post_process(args)
        else:
            data = geo_point.data
            p_from = self.get_proj(data)
            p_to = to_geo_type.get_proj(data)
            args = self.getargs(geo_point)
            out = pyproj.transform(p_from, p_to, args[0], args[1])
            array_out = np.array(out)  # just in case its not a numpy already, and will simplify calcs later
        return to_geo_type.post_process(array_out)

    def post_process(self, transformed_points):
        return self.reorder(transformed_points)

    def reorder(self, elements):
        array_elements = np.array(elements)
        if len(array_elements.shape) <= 1:
            post_out = array_elements[self.proj_transform_order]
        else:
            post_out = array_elements[self.proj_transform_order, :]
        return post_out

    def getargs(self, geo_point):
        parameters = self.reorder(self.values)
        return geo_point.data[parameters[0]], geo_point.data[parameters[1]]


class LatLon(GeoType):
    def __init__(self):
        super(LatLon, self).__init__("latlon", ["latitude", "longitude"], {"proj": "latlong"},
                                     ["longitude", "latitude"])

    def transform(self, geo_point, to_geo_type):
        if to_geo_type.name == "utm":
            np_longitude = np.array(geo_point.data["longitude"])
            zones = (((np_longitude + 180).round() / 6.0) % 60 + 1).astype(int)
            geo_point.data["zonenum"] = zones[0] if isinstance(zones, np.ndarray) else zones
            out = super(LatLon, self).transform(geo_point, to_geo_type)
            a = list(out)
            a.append(zones)
            out = a
        else:
            out = super(LatLon, self).transform(geo_point, to_geo_type)

        return out

UTM = GeoType("utm", ["easting", "northing", "zonenum"], {"proj": "utm", "zone": "zonenum"}, ["easting", "northing"])
LAT_LONG = LatLon()


# realization: utm and latlong are "primitive" geoTypes. DEMType has a different structure,
# but still same interface as the other geoTypes. It also relies on geo_points interface
class DEMType(GeoType):
    def __init__(self, origin, resolution):
        super(DEMType, self).__init__("coord", ["x", "y", "zonenum"], {"proj": "utm", "zone": "zonenum"}, ["x", "y"])
        self.projtype = "utm"

        # doing conversion early on will save use from redoing it later, we don't expect our origin to change too much
        self.origin_easting, self.origin_northing, self.origin_zone = np.array(origin.to(UTM))
        self.resolution = resolution

    def get_proj(self, geo_points):
        return super(DEMType, self).get_proj({"zonenum": self.origin_zone})

    # this function is used internally and externally
    def getargs(self, geo_points):
        # next line should ideally be super.getargs, but we overwrite the fx so not sure if possible
        x, y = geo_points.data["x"], geo_points.data["y"]
        delta_easting, delta_northing = np.array([x, y]) * self.resolution
        return self.origin_easting + delta_easting, self.origin_northing - delta_northing

    def post_process(self, transformed_points):
        points_easting, points_northing = transformed_points
        x, y = (points_easting - self.origin_easting, self.origin_northing - points_northing)
        return np.array([np.floor(x / self.resolution), np.floor(y / self.resolution)]).astype(int)

import folium
import pandas
class GeoPoint(object):
    def __init__(self, geo_type, *args):
        self.data = dict()

        counter = 0

        # allow for a couple of input options:
        if isinstance(geo_type, list):
            self.geo_type = LAT_LONG
            # assume list of points, convert to lat long
            self.data["latitude"], self.data["longitude"] = [], []
            for points in geo_type:
                lat, long = points.to(LAT_LONG)
                self.data["latitude"].append(lat)
                self.data["longitude"].append(long)
        else:
            self.geo_type = geo_type
            if len(args) == 1:
                # python pandas library
                pandalist = args[0]
                for entry in geo_type.values:
                    self.data[entry] = pandalist[entry].values
            else:
                for arg in args:
                    # print geo_type.values[counter]
                    self.data[geo_type.values[counter]] = arg
                    # print arg
                    counter += 1

    def disp(self):
        for k, v in self.data.iteritems():
            print "%s : %s" % (k, v)

    def values(self):
        # need to make immutable
        values = []
        for parameter in self.geo_type.values:
            values.append(self.data[parameter])
        return values

    def to(self, to_geo_type):
        return self.geo_type.transform(self, to_geo_type)

    def upper_left(self):
        easting, northing, zones = self.to(UTM)
        easting_upper_left = easting.min()
        northing_upper_left = northing.max()
        return GeoPoint(UTM, easting_upper_left, northing_upper_left, zones[0])

    def square(self):
        easting, northing, zones = self.to(UTM)
        minx = easting.min()
        miny = northing.min()
        maxx = easting.max()
        maxy = northing.max()
        xcoords = np.array([minx, minx, maxx, maxx])
        ycoords = np.array([maxy, miny, miny, maxy])
        # TODO: remove hack from zones
        return GeoPoint(UTM, xcoords, ycoords, zones[0])

    def middle(self):
        easting, northing, zones = self.to(UTM)
        easting_mean, northing_mean = np.mean([easting, northing], axis=0)
        return GeoPoint(UTM, easting_mean, northing_mean, zones)