import requests
import json
import sys
sys.path.append('../')
from pextant.analysis.loadWaypoints import JSONloader
WP_HI = JSONloader.from_file('../data/waypoints/HI_13Nov16_MD7_A.json')

xp_json = WP_HI.sequence

r1 = requests.post('https://localhost/pextant/setwaypoints',
                   data = json.dumps(xp_json), verify=False, timeout=5)
print(r1.json())
r2 = requests.post('https://localhost/pextant/solve',
                   data = json.dumps({}), verify=False,)
json_response = r2.json()
print(json_response["latitudes"])