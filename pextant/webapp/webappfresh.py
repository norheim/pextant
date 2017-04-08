from flask import Flask
from flask_socketio import SocketIO
from serialgps import GPSSerialThread, serial_ports, GPSSerialEmulator, RandomThread
from pextant.lib.geoshapely import LAT_LONG, GeoPoint, Cartesian, UTM
from pextant.analysis.loadWaypoints import JSONloader

import json
from threading import Thread

app = Flask(__name__)
socketio = SocketIO(app)

thread = Thread()   #needed for gps stream

@socketio.on('connect')
def handle_new_connection():
    def message(message):
        print(message)
    messenger = SocketChannel(socketio, 'message', message)
    messenger.emit('connected1')
    print('Client connected')

@socketio.on('serialstatus')
def serialstatus(data):
    print('got serial data request')
    returnstring = json.dumps(serial_ports())
    print returnstring
    socketio.emit('event', returnstring)
    socketio.emit('serialstatus', returnstring)

@socketio.on('gpsmesh')
def gpsmesh(data):
    print data

@socketio.on('gpstrack')
def gpstrack_handler(data):
    print 'requesting gps tracking'
    socketio.emit('message', 'turning gps on')
    global thread
    if not thread.isAlive():
        print "Starting Thread"
        #thread = GPSSerialEmulator(SocketChannel(socketio, 'gpstrack'), SocketChannel(socketio, 'gpsmesh'), 'COM5')
        thread.start()
    else:
        thread.record = True
        socketio.emit('message', 'gps already on')

@socketio.on('gpstracksilencer')
def gpstrack_stopper(data):
    print 'silencing gps'
    socketio.emit('message', 'silencing gps')
    global thread
    if thread.isAlive():
        print thread.record
        thread.record = False
        thread.stop()

class SocketChannel:
    def __init__(self, socket, channelname, onrecieve):
        self.channelname = channelname
        self.socket = socket
        self.onrecieve = socketio.on(channelname)(onrecieve)

    def emit(self, message):
        print 'sending message'
        self.socket.emit(self.channelname, message)

#meshmessenger = SocketChannel(socketio, 'meshmsg')

@socketio.on('waypoints')
def getwaypoints(data):
    print('got waypoint request')
    print(data != "")
    waypoints = JSONloader('../../data/waypoints/HI_13Nov16_MD7_A.json').get_waypoints()

    waypointsdict = {
        'latitude': list(waypoints[0]),
        'longitude' : list(waypoints[1])}

    print waypointsdict
    waypointsstr = json.dumps(waypointsdict)
    print waypointsstr
    socketio.emit('waypoints', waypointsstr)

@socketio.on('calibrate')
def getcalibration(data):
    print('got calibration request')
    global thread
    global GPS_OFFSET
    mappoint = json.loads(data)
    print data
    gpspoint = json.loads(thread.most_recent_gps_point_raw)
    print gpspoint
    geomappoint = GeoPoint(LAT_LONG, mappoint["latitude"], mappoint["longitude"])
    geogpspoint = GeoPoint(LAT_LONG, gpspoint["latitude"], gpspoint["longitude"])
    print geogpspoint
    print geomappoint
    offset = geogpspoint.to(UTM(5)) - geomappoint.to(UTM(5))
    print offset
    GPS_OFFSET = {
        'easting': offset[0],
        'northing': offset[1]
    }
    thread.gpsoffset = GPS_OFFSET
    socketio.emit('message', 'calibration completed')
    socketio.emit('message', json.dumps(GPS_OFFSET))

@socketio.on('pextant')
def getpextant(data):
    print('got pextant request')
    global thread
    global meshmessenger
    waypoint = None
    #if data != "":
    #waypoint0 = json.loads(thread.most_recent_gps_point)
    #waypoint1 = json.loads(data)
    #print waypoint1['latitude']
    #print waypoint0['latitude']
    #waypoint =  json.dumps({"latitude":[waypoint0['latitude'], waypoint1['latitude']],
    #             "longitude": [waypoint0['longitude'], waypoint1['longitude']]})
    #print waypoint
        #waypoints = runpextant(meshmessenger)
    #else:

        #waypoints = runpextant(meshmessenger)


    #print waypoints
    #waypointsdict = {
    #    'latitude': list(waypoint[0]),
    #    'longitude': list(waypoint[1])}

    #print waypointsdict
    #waypointsstr = json.dumps(waypointsdict)
    #print waypointsstr
    socketio.emit('pextant', waypointsstr)

@socketio.on('disconnect')
def disconnect():
    global thread
    if thread.isAlive():
        thread.stop()
    print('disconnected')

if __name__ == '__main__':
    socketio.run(app, host='localhost', port=2999)