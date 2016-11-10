import pynmea2
import serial
import sys
from serial.tools import list_ports

def serial_ports():
    for connection in list_ports.comports():
        print(connection.description)

def serialRun():
    ser = serial.Serial()
    ser.port = "COM4"
    ser.baudrate = 4800
    ser.timeout = 1
    ser.open()
    streamreader = pynmea2.NMEAStreamReader()

    while True:
        data = ser.readline()
        print data
        if data[0:6] == '$GPGGA':
            msg = pynmea2.parse(data)
            lats = msg.latitude
            lon = msg.longitude
            print "("+str(lats)+","+str(lon)+")"