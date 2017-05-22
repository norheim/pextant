import requests
import json
import sys
sys.path.append('../')
from pextant.analysis.loadWaypoints import JSONloader
WP_HI = JSONloader.from_file('../data/waypoints/HI_13Nov16_MD7_A.json')

xp_json = WP_HI.sequence

r1 = requests.post('http://localhost:5000/setwaypoints', data = json.dumps(xp_json), timeout=5)
print(r1.json())
r2 = requests.post('http://localhost:5000/solve', data = json.dumps({}))
json_response = r2.json()
print(json_response["latitudes"])