from api import *
from EnvironmentalModel import *
from ExplorerModel import Astronaut
from loadWaypoints import loadPoints

def runpextant(filename, pointJSON=None):
    waypoints = loadPoints(filename)
    if pointJSON:
        point = json.loads(pointJSON)
        lat, lon = waypoints.to(LAT_LONG)
        waypoints = GeoPolygon(LAT_LONG, [point['latitude']]+list(lat), [point['longitude']]+list(lon))
        print waypoints.to(LAT_LONG)
    aps =[]
    for waypoint in waypoints:
        aps.append(ActivityPoint(waypoint, 0))

    dem_path = 'maps/HI_lowqual_DEM.tif'
    dataset, info = loadElevationsLite(dem_path)
    resolution = info["resolution"]
    XY = Cartesian(info["nw_geo_point"], resolution)
    nw_corner, se_corner = waypoints.geoEnvelope().addMargin(XY,10).getBounds()
    EM2 = loadElevationMap(dem_path, maxSlope=35, planet='Earth', nw_corner=nw_corner, se_corner=se_corner,
                              desired_res=resolution, no_val=-10000)
    astronaut = Astronaut(45)
    P = Pathfinder(astronaut, EM2)

    out, dummy1, dummy2  = P.aStarCompletePath([0,0,1], aps, 'tuple')
    npout= np.array(out).transpose()
    lat_long_out = GeoPolygon(EM2.ROW_COL,npout[0],npout[1]).to(LAT_LONG)
    return  lat_long_out

if __name__ == '__main__':
    print('got waypoint request')
    waypoints = runpextant('waypoints/HI_sextant_testing2_B.json',
                           '{"latitude":19.36477996833597, "longitude":-155.2008308586817}')
    print waypoints
    waypointsdict = {
        'latitude': list(waypoints[0]),
        'longitude' : list(waypoints[1])}

    print waypointsdict
    waypointsstr = json.dumps(waypointsdict)
    print waypointsstr