from time import time, sleep
from threading import Thread, Event
import serial
import pynmea2
import json
from serial.tools import list_ports
from geoshapely import GeoPoint, LAT_LONG, Cartesian, UTM
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


class GPSRecorderThread(StoppableThread):
    def __init__(self, socket_channel):
        super(GPSRecorderThread, self).__init__()
        self.socket_channel = socket_channel
        self.gpsoffset = {
            'easting': 0,
            'northing': 0
        }
        self.recorded_points = {}
        self.delay = 0.1
        self.record = True
        self.most_recent_gps_point = None
        self.most_recent_gps_point_raw = None
        self.save_name_baseline = str(time())

    def getSaveName(self, extraname):
        return 'gps/gps_' + extraname + '_' + self.save_name_baseline + '.json'

    def correctForOffset(self, lat, long):
        easting, northing = GeoPoint(LAT_LONG, lat, long).to(UTM(5))
        print self.gpsoffset["easting"]
        print self.gpsoffset['northing']
        trueasting = easting - self.gpsoffset["easting"]
        truenorthing = northing - self.gpsoffset['northing']
        return GeoPoint(UTM(5), trueasting, truenorthing).to(LAT_LONG)

    def newGPSPoint(self, lat, lon, alt):
        print lat, lon
        raw_point_json = self.recordAndSaveGPSPoint(lat, lon, alt, 'raw')
        correctedPointLat, correctedPointLong = self.correctForOffset(lat, lon)
        corrected_point_json = self.recordAndSaveGPSPoint(correctedPointLat, correctedPointLong, 0, 'corrected')
        self.most_recent_gps_point = corrected_point_json
        self.most_recent_gps_point_raw = raw_point_json
        self.emit(corrected_point_json)

    def recordAndSaveGPSPoint(self, lat, lon, alt, extraname=''):
        latlon_point = {
            'time': time(),
            'latitude': lat,
            'longitude': lon,
            'altitude': 0}
        if extraname in self.recorded_points:
            self.recorded_points[extraname].append(latlon_point)
        else:
            self.recorded_points[extraname] = [latlon_point]

        savefile_name = self.getSaveName(extraname)
        with open(savefile_name, 'w') as outfile:
            json.dump(self.recorded_points[extraname], outfile)

        latlon_point_json = json.dumps(latlon_point)
        return latlon_point_json

    def emit(self, latlon_point_json):
        print str(time())
        self.socket_channel.emit(latlon_point_json)


class GPSSerialThread(GPSRecorderThread):
    def __init__(self, socket_channel, comport):
        super(GPSSerialThread, self).__init__(socket_channel)
        self.comport = comport
        self.baudrate = 4800
        self.serial_reference = None

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
                if self.record:
                    self.newGPSPoint(msg.latitude, msg.longitude, msg.altitude)

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


class GPSSerialEmulator(GPSRecorderThread):
    def __init__(self, socket_channel, comport):
        self.time = time()
        super(GPSSerialEmulator, self).__init__(socket_channel)

    def randomPointGenerator(self):
        while not self.stopped():
            newtime = time()
            deltat = newtime - self.time
            if deltat > 1:
                self.time = newtime
                center = GeoPoint(LAT_LONG, 19.36479555, -155.20178273)
                XY = Cartesian(center, 1)
                randomPointLat, randomPointLong = GeoPoint(XY, randint(0, 100), randint(0, 100)).to(LAT_LONG)
                self.newGPSPoint(randomPointLat, randomPointLong, 0)

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
