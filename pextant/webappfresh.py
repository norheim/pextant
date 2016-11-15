from flask import Flask
from flask_socketio import SocketIO
from serialgps import GPSSerialThread, serial_ports, GPSSerialEmulator, RandomThread
from geoshapely import LAT_LONG
from loadWaypoints import loadPoints
from hawaiiclean import runpextant

import json
from threading import Thread

app = Flask(__name__)
socketio = SocketIO(app)

thread = Thread()   #needed for gps stream

@socketio.on('connect')
def handle_new_connection():
    messenger = SocketChannel(socketio, 'message')
    messenger.emit('connected1')
    print('Client connected')

@socketio.on('serialstatus')
def serialstatus(data):
    print('got serial data request')
    returnstring = json.dumps(serial_ports())
    print returnstring
    socketio.emit('event', returnstring)
    socketio.emit('serialstatus', returnstring)

@socketio.on('gpstrack')
def gpstrack_handler(data):
    print 'requesting gps tracking'
    socketio.emit('message', 'turning gps on')
    global thread
    if not thread.isAlive():
        print "Starting Thread"
        thread = GPSSerialEmulator(SocketChannel(socketio, 'gpstrack'), 'COM6')
        thread.start()
    else:
        thread.record = True

@socketio.on('gpstracksilencer')
def gpstrack_stopper(data):
    print 'silencing gps'
    socketio.emit('message', 'silencing gps')
    global thread
    if thread.isAlive():
        print thread.record
        thread.record = False

class SocketChannel:
    def __init__(self, socket, channelname):
        self.channelname = channelname
        self.socket = socket

    def emit(self, message):
        self.socket.emit(self.channelname, message)

@socketio.on('waypoints')
def getwaypoints(data):
    print('got waypoint request')
    waypoints = loadPoints('waypoints/HI_13Nov16_MD7_A.json').to(LAT_LONG)
    print waypoints
    waypointsdict = {
        'latitude': list(waypoints[0]),
        'longitude' : list(waypoints[1])}

    print waypointsdict
    waypointsstr = json.dumps(waypointsdict)
    print waypointsstr
    socketio.emit('waypoints', waypointsstr)

@socketio.on('pextant')
def getpextant(data):
    print('got pextant request')
    global thread
    waypoint = None
    if data == 'gps':
        waypoint = thread.most_recent_gps_point
    waypoints = runpextant('waypoints/HI_13Nov16_MD7_A.json', waypoint)
    print waypoints
    waypointsdict = {
        'latitude': list(waypoints[0]),
        'longitude': list(waypoints[1])}

    print waypointsdict
    waypointsstr = json.dumps(waypointsdict)
    print waypointsstr
    socketio.emit('pextant', waypointsstr)

@socketio.on('message')
def message(message):
    print(message)

@socketio.on('disconnect')
def disconnect():
    print('disconnected')

if __name__ == '__main__':
    socketio.run(app, host='localhost', port=2999)