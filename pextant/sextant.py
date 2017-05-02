from flask_settings import GEOTIFF_FULL_PATH
import sys
sys.path.append('../')
import numpy as np
import json
from datetime import timedelta

from functools import update_wrapper

from pextant.EnvironmentalModel import GDALMesh
from pextant.analysis.loadWaypoints import JSONloader
from pextant.lib.geoshapely import GeoPolygon, LAT_LONG
from pextant.solvers.astarMesh import astarSolver

from flask import Flask
from flask import make_response, request, current_app
app = Flask(__name__)

def crossdomain(origin=None, methods=None, headers=None,
                max_age=21600, attach_to_all=True,
                automatic_options=True):
    if methods is not None:
        methods = ', '.join(sorted(x.upper() for x in methods))
    if headers is not None and not isinstance(headers, basestring):
        headers = ', '.join(x.upper() for x in headers)
    if not isinstance(origin, basestring):
        origin = ', '.join(origin)
    if isinstance(max_age, timedelta):
        max_age = max_age.total_seconds()

    def get_methods():
        if methods is not None:
            return methods

        options_resp = current_app.make_default_options_response()
        return options_resp.headers['allow']

    def decorator(f):
        def wrapped_function(*args, **kwargs):
            if automatic_options and request.method == 'OPTIONS':
                resp = current_app.make_default_options_response()
            else:
                resp = make_response(f(*args, **kwargs))
            if not attach_to_all and request.method != 'OPTIONS':
                return resp

            h = resp.headers

            h['Access-Control-Allow-Origin'] = origin
            h['Access-Control-Allow-Methods'] = get_methods()
            h['Access-Control-Max-Age'] = str(max_age)
            if headers is not None:
                h['Access-Control-Allow-Headers'] = headers
            return resp

        f.provide_automatic_options = False
        return update_wrapper(wrapped_function, f)
    return decorator

def main(argv):
    try:
        geotiff_full_path = argv[0]
    except IndexError:
        # print 'Syntax is "sextant <inputfile>"'
        geotiff_full_path = GEOTIFF_FULL_PATH

    gdal_mesh = GDALMesh(geotiff_full_path)

    @app.route('/', methods=['GET', 'POST'])
    @crossdomain(origin='*')
    def set_waypoints():
        print('got request')
        data = request.get_json(force=True)
        data_np = np.array(data['waypoints']).transpose()
        #json_waypoints = JSONloader(xpjson)
        waypoints = GeoPolygon(LAT_LONG, *data_np)
        print waypoints.to(LAT_LONG)

        environmental_model = gdal_mesh.loadSubSection()
        #solver = astarSolver(environmental_model, explorer, optimize_on='Energy')
        #search_results, rawpoints, itemssrchd = solver.solvemultipoint(waypoints)
        return json.dumps({'status': 'OK', 'shape': environmental_model.size})

    app.run(host='localhost', port=5000)


if __name__ == "__main__":
    main(sys.argv[1:])