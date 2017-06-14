import requests
import json
import sys
sys.path.append('../')
from pextant.analysis.loadWaypoints import JSONloader
WP_HI = JSONloader.from_file('../data/waypoints/HI_13Nov16_MD7_A.json')

xp_json = WP_HI.sequence

config1 = 'https://localhost/pextant'
config2 = 'http://localhost:5000'

send_json1 = {
    'param': {},
    'xp_json': xp_json
}

r1 = requests.post(config2 + '/setwaypoints',
                   data = json.dumps(send_json1), verify=False, timeout=5)
print(r1.json())
send_json2 = {
    'param': {
        'return':'segmented'
    },
}
r2 = requests.post(config2+'/solve',
                   data = json.dumps(send_json2), verify=False,)
json_response = r2.json()
print(json_response)