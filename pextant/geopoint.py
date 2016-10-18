import pyproj

class GeoType(object):
    def __init__(self, name, values, projparam, projtransformorder):
        self.name = name
        self.values = values
        self.projparam = projparam
        self.proj_transform_order = projtransformorder

    def getProj(self, geo_point):
        proj = self.projparam["proj"]

        for k, v in self.projparam.iteritems():
            for val in self.values:
                self.projparam[k] = geo_point.data[v] if v == val else v

        # add additional parameters here
        self.projparam["datum"] = "WGS84"

        proj_param = self.projparam;
        p = pyproj.Proj(**proj_param)

        return p

    # will belong to all instance
    def transform(self, geo_point, to_geo_type):
        out1, out2 = self.transform_implementation(geo_point, to_geo_type)
        out3, out4 = to_geo_type.postprocess((out1, out2))
        return out3, out4

    # will be specific to certain instances
    def transform_implementation(self, geo_point, to_geo_type):
        p_from = self.getProj(geo_point)
        p_to = to_geo_type.getProj(geo_point)
        # need to add function that is specific to geo_type. Problem comes from dealing with coord type values
        # x, y = geo_point.data[self.values[0]], geo_point.data[self.values[1]] # TODO: make function for this?
        x, y = self.getargs(geo_point)

        out1, out2 = pyproj.transform(p_from, p_to, x, y)

        return out1, out2

    def postprocess(selg, transformed_points):
        return transformed_points

    def getargs(self, geo_point):
        return self.getargshelper(geo_point)

    def getargshelper(self, geo_point):
        return geo_point.data[self.proj_transform_order[0]], geo_point.data[self.proj_transform_order[1]]


import numpy as np
from math import floor


# realization: utm and latlong are "primitive" geoTypes. DEMType has a different structure,
# but still same interface as the other geoTypes. It also relies on geo_points interface
class DEMType(GeoType):
    # needed for all coordinate conversions. got some serious bootstrapping going on right here
    utm = GeoType("utm", ["easting", "northing", "zonenum"], {"proj": "utm", "zone": "zonenum"},
                  ["easting", "northing"])

    def __init__(self, origin, resolution):
        self.name = "coord"
        self.values = ['x', 'y']
        # doing conversion early on will save use from redoing it later, we don't expect our origin to change too much
        self.origin = np.array(origin.to(utm))
        self.projparamt = []
        self.resolution = resolution

    # TODO: should be able to clean transform function even more
    def getProj(self, geo_points):
        return utm.getProj(geo_points)

    # this function is used internally and externally
    def getargs(self, geo_points):
        origin_easting, origin_northing = self.origin
        # next line should ideally be super.getargs, but we overwrite the fx so not sure if possible
        x, y = geo_points.data[geo_points.geo_type.values[0]], geo_points.data[geo_points.geo_type.values[1]]
        points_easting, points_northing = origin_easting + x, origin_northing - y
        return points_easting, points_northing

    def postprocess(self, transformed_points):
        origin_easting, origin_northing = self.origin
        points_easting, points_northing = transformed_points
        x, y = (points_easting - origin_easting, origin_northing - points_northing)
        return np.floor(x / self.resolution), np.floor(y / self.resolution)

    def transform_implementation(self, geo_points, to_geo_type):
        x, y = geo_point.data[self.values[0]], geo_point.data[self.values[1]]

        # if we didnt do conversion eaerlier this is where we would have to do it
        origin_easting, origin_northing = self.origin
        points_easting, points_northing = origin_easting + x, origin_northing - y

        return points_easting, points_northing


class GeoPoint(object):
    def __init__(self, geo_type, *args):
        self.geo_type = geo_type
        self.data = dict()

        counter = 0

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

    def raw(self):
        return self.data.values()

    def values(self):
        # need to make immutable
        return self.data.copy()

    def to(self, to_geo_type):
        if self.geo_type != to_geo_type:
            out1, out2 = self.geo_type.transform(self, to_geo_type)
        else:
            return self.raw()

        return np.array((out1, out2))

    def square(self):
        easting, northing = self.to(utm)
        minx = easting.min()
        miny = northing.min()
        maxx = easting.max()
        maxy = northing.max()
        xcoords = np.array([minx, minx, maxx, maxx])
        ycoords = np.array([maxy, miny, miny, maxy])
        return GeoPoint(utm, xcoords, ycoords)

utm = GeoType("utm", ["easting", "northing", "zonenum"], {"proj": "utm", "zone": "zonenum"}, ["easting", "northing"])
latlong = GeoType("latlon", ["latitude", "longitude"], {"proj": "latlong"}, ["longitude", "latitude"])