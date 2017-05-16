import requests
import json
ames_data = {
            "waypoints": [
                [  37.42015095, -122.06505988],
                [  37.42014872, -122.06477738]
            ],
            "time": "2pm"
        }

r = requests.post('http://localhost:5000/', data = json.dumps(ames_data))
json_response = r.json()
print(json_response["latitudes"])