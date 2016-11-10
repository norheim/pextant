import socketio
import eventlet
import pandas as pd
import json
import time
import pynmea2
import serial
from gpsstream import *
from loadWaypoints import loadPoints
from geoshapely import *
from hawaiiclean import runpextant
import threading

sio = socketio.Server()

d =  {'time': [], 'latitude' : [],'longitude' : []}
gps_on = False
threads = []

def save():
    print 'save'
    pdd = pd.DataFrame({'time': pd.Series(d["time"]), 'latitude': pd.Series(d["latitude"]),
           'longitude': pd.Series(d["longitude"])})
    timestring = time.strftime("%Y%m%d_%H%M%S")
    savestring = 'gps/gps_'+str(timestring)+'.csv'
    print savestring
    print pdd
    pdd.to_csv(savestring,index=False)

def gps_status():
    return gps_on

@sio.on('connect')
def connect(sid, environ):
    print('turning gps on, listening on COM port:')
    comport = 'COM6'
    print comport
    channelname = 'gpstrack'
    t = threading.Thread(target=serialRun, args=(comport, sio, channelname))
    threads.append(t)
    t.start()
    print('connect ', sid)

@sio.on('message')
def message(sid, data):
    print('message ', data)
    sio.emit('event', data)

@sio.on('serialstatus')
def serialstatus(sid, data):
    print('got serial data request')
    returnstring = json.dumps(serial_ports())
    print returnstring
    #sio.emit('event', returnstring)
    sio.emit('serialstatus', returnstring)

def helperfx(data):
    print(data)

@sio.on('gpstrack')
def gpstrack(sid, data):
    channelname = 'gpstrack'
    #if not gps_status():
    sio.emit('event', 'gps is on')

@sio.on('waypoints')
def getwaypoints(sid,data):
    print('got waypoint request')
    waypoints = loadPoints('waypoints/HI_sextant_testing2_B.json').to(LAT_LONG)
    print waypoints
    waypointsdict = {
        'latitude': list(waypoints[0]),
        'longitude' : list(waypoints[1])}

    print waypointsdict
    waypointsstr = json.dumps(waypointsdict)
    print waypointsstr
    sio.emit('waypoints', waypointsstr)

@sio.on('pextant')
def getpextant(sid,data):
    print('got pextant request')
    waypoints = runpextant('waypoints/HI_sextant_testing2_B.json')
    print waypoints
    waypointsdict = {
        'latitude': list(waypoints[0]),
        'longitude': list(waypoints[1])}

    print waypointsdict
    waypointsstr = json.dumps(waypointsdict)
    print waypointsstr
    sio.emit('pextant', waypointsstr)

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

@sio.on('disconnect')
def disconnect(sid):
    print('disconnect ', sid)

if __name__ == '__main__':
    # wrap Flask application with socketio's middleware
    app = socketio.Middleware(sio)
    print('listening...')
    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('localhost', 3000)), app)