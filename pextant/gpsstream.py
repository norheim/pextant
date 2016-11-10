import pynmea2
import serial
import json
import time
import pandas as pd
from serial.tools import list_ports

d =  {'time': [], 'latitude' : [],'longitude' : []}

def serial_ports():
    com_ports = []
    for connection in list_ports.comports():
        com_ports.append(connection.description)
    return com_ports

def serialRun(portname, sio=None, channelname=None):
    ser = serial.Serial()
    ser.port = portname
    ser.baudrate = 4800
    ser.timeout = 1
    ser.open()
    allpoints = []
    timestring = time.strftime("%Y%m%d_%H%M%S")
    while True:
        data = ser.readline()
        print data[0:6]
        if data[0:6] == '$GPGGA':
            msg = pynmea2.parse(data)
            lats = msg.latitude
            lon = msg.longitude
            print "("+str(lats)+","+str(lon)+")"
            obj = {'latitude': lats, 'longitude': lon}
            allpoints.append(obj)

            timesec, lat, lon = time.time(), lats, lon
            d["time"].append(timesec)
            d["latitude"].append(lat)
            d["longitude"].append(lon)
            pdd = pd.DataFrame({'time': pd.Series(d["time"]), 'latitude': pd.Series(d["latitude"]),
                                'longitude': pd.Series(d["longitude"])})
            savestring = 'gps/gps_' + str(timestring) + '.csv'
            print savestring
            pdd.to_csv(savestring, index=False)

            jsonobj = json.dumps(obj)
            print('json:')
            print(jsonobj)
            sio.emit('event', jsonobj)
            print('sent message on channel:')
            print channelname


#serialRun('COM6')