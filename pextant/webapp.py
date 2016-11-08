import socketio
import eventlet
import pandas as pd
import json
import time

sio = socketio.Server()

d =  {'time': [], 'latitude' : [],'longitude' : []}

def save():
    print 'save'
    pdd = pd.DataFrame({'time': pd.Series(d["time"]), 'latitude': pd.Series(d["latitude"]),
           'longitude': pd.Series(d["longitude"])})
    timestring = time.strftime("%Y%m%d_%H%M%S")
    savestring = 'gps/gps_'+str(timestring)+'.csv'
    print savestring
    print pdd
    pdd.to_csv(savestring,index=False)

@sio.on('connect')
def connect(sid, environ):

    print('connect ', sid)

@sio.on('message')
def message(sid, data):
    print('message ', data)
    sio.emit('event', data)

@sio.on('coords')
def coords(sid, data):
    print('appending coordinates', data)
    coords = json.loads(data)
    time, lat, lon = coords['time'], coords['latitude'], coords['longitude']
    d["time"].append(time)
    d["latitude"].append(lat)
    d["longitude"].append(lon)
    #sio.emit('event', 'saved data')

@sio.on('stop')
def message(sid, data):
    print('stopped')
    save()

@sio.on('disconnect')
def disconnect(sid):
    save()
    print('disconnect ', sid)

if __name__ == '__main__':
    # wrap Flask application with socketio's middleware
    app = socketio.Middleware(sio)
    print('listening...')
    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('localhost', 3000)), app)