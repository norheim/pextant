from time import time, sleep
from threading import Thread, Event
import serial
import pynmea2
import json
from serial.tools import list_ports
import eventlet
eventlet.monkey_patch(thread=True)

thread_stop_event = Event()

def serial_ports():
    com_ports = []
    for connection in list_ports.comports():
        com_ports.append(connection.description)
    return com_ports

class GPSSerialThread(Thread):
    def __init__(self, socket_channel, comport):
        super(GPSSerialThread, self).__init__()
        self.socket_channel = socket_channel
        self.comport = comport
        self.baudrate = 4800
        self.recorded_points_file_name = 'gps/gps_' + str(time()) + '.json'
        self.recorded_points = []
        self.delay = 0.1
        self.serial_reference = None

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
        while not thread_stop_event.isSet():
            data = self.serial_reference.readline()
            if data[0:6] == '$GPGGA':
                msg = pynmea2.parse(data)
                latlon_point = {
                    'time': time(),
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude}
                latlon_point_json = json.dumps(latlon_point)
                print latlon_point
                self.recorded_points.append(latlon_point)
                self.saveRecordedPoints()
                self.socket_channel.emit(latlon_point_json)
                eventlet.sleep()

    def run(self):
        self.streamFromSerial()

class RandomThread(Thread):
    def __init__(self, socket_ref):
        self.socket_ref = socket_ref
        self.delay = 1
        super(RandomThread, self).__init__()

    def randomNumberGenerator(self):
        while not thread_stop_event.isSet():
            print 'emitting'
            self.socket_ref.emit('event', str(time()))
            eventlet.sleep()

    def run(self):
        self.randomNumberGenerator()

class SocketTest(Thread):
    def __init__(self, socket_ref):
        self.socket_ref = socket_ref
        self.delay = 1
        super(SocketTest, self).__init__()

    def greeter(self):
        while not thread_stop_event.isSet():
            print 'emitting'
            self.socket_ref.emit('Greetings')
            eventlet.sleep()

    def run(self):
        self.greeter()

class FakeEmitter(object):
    def __init__(self):
        pass

    def emit(self, message):
        print 'emitting'
        print message

if __name__ == '__main__':
    gps = GPSSerialThread(FakeEmitter(), 'COM6')
    gps.streamFromSerial()