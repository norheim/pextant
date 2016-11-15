from time import time, sleep
from threading import Thread, Event
import serial
import pynmea2
import json
from serial.tools import list_ports
from geoshapely import GeoPoint, LAT_LONG, Cartesian
from random import randint
import eventlet
eventlet.monkey_patch(thread=True)

def serial_ports():
    com_ports = []
    for connection in list_ports.comports():
        com_ports.append(connection.description)
    return com_ports

class StoppableThread(Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self):
        super(StoppableThread, self).__init__()
        self._stop = Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

class GPSSerialThread(StoppableThread):
    def __init__(self, socket_channel, comport):
        super(GPSSerialThread, self).__init__()
        self.socket_channel = socket_channel
        self.comport = comport
        self.baudrate = 4800
        self.recorded_points_file_name = 'gps/gps_' + str(time()) + '.json'
        self.recorded_points = []
        self.delay = 0.1
        self.serial_reference = None
        self.record = True
        self.most_recent_gps_point = None

    def saveRecordedPoints(self):
        with open(self.recorded_points_file_name, 'w') as outfile:
            json.dump(self.recorded_points, outfile)

    def openSerialConnection(self):
        ser = serial.Serial()
        ser.port = self.comport
        ser.baudrate = self.baudrate
        ser.timeout = 1
        ser.open()
        self.serial_reference = ser

    def streamFromSerial(self):
        self.openSerialConnection()
        while not self.stopped():
            data = self.serial_reference.readline()
            if data[0:6] == '$GPGGA':
                msg = pynmea2.parse(data)
                latlon_point = {
                    'time': time(),
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude}
                latlon_point_json = json.dumps(latlon_point)
                self.most_recent_gps_point = latlon_point_json
                print latlon_point
                if self.record:
                    self.recorded_points.append(latlon_point)
                    self.saveRecordedPoints()
                    self.socket_channel.emit(latlon_point_json)
                eventlet.sleep()

    def run(self):
        self.streamFromSerial()

class RandomThread(StoppableThread):
    def __init__(self, socket_ref):
        self.socket_ref = socket_ref
        self.delay = 1
        self.record = True
        super(RandomThread, self).__init__()

    def randomNumberGenerator(self):
        while not self.stopped():
            print 'emitting'
            self.socket_ref.emit('event', str(time()))
            eventlet.sleep()

    def run(self):
        self.randomNumberGenerator()

class GPSSerialEmulator(StoppableThread):
    def __init__(self, socket_channel, comport):
        self.socket_channel = socket_channel
        self.time = time()
        self.record = True
        self.most_recent_gps_point = None
        self._stop = Event()
        super(GPSSerialEmulator, self).__init__()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def randomPointGenerator(self):
        while not self.stopped():
            #print 'emitting'
            newtime = time()
            deltat = newtime-self.time
            #print deltat
            if deltat > 1:
                self.time = newtime
                center = GeoPoint(LAT_LONG, 19.36479555, -155.20178273)
                XY = Cartesian(center, 1)
                randomPointLat, randomPointLong = GeoPoint(XY,randint(0,100), randint(0,100)).to(LAT_LONG)
                latlon_point = {
                    'time': time(),
                    'latitude': randomPointLat,
                    'longitude': randomPointLong,
                    'altitude': 0}
                latlon_point_json = json.dumps(latlon_point)
                self.most_recent_gps_point = latlon_point_json
                print latlon_point
                if self.record:
                    self.socket_channel.emit(latlon_point_json)
            eventlet.sleep()

    def run(self):
        self.randomPointGenerator()

class FakeEmitter(object):
    def __init__(self):
        pass

    def emit(self, message):
        print 'emitting'
        print message

if __name__ == '__main__':
    gps = GPSSerialEmulator(FakeEmitter(), 'COM6')
    gps.start()