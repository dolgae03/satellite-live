import socket
import struct
import time
import math
import numpy as np
from skyfield.api import load, wgs84


def time_packet(year, month, day, hour, min, sec):

    return struct.pack('BBBiBBBBB',0x96, 0x00, 0x00,year, month, day, hour, min, sec)

def attitude_packet(quaternion):
    q1, q2, q3, q4 = quaternion

    return struct.pack('BBBffff',0x96,0x00,0x02, q1, q2, q3, q4)

def location_packet(location):
    x, y, z = location

    print(struct.unpack('BBBfff',struct.pack('BBBfff',0x96,0x00,0x01,x,y,z)))

    return struct.pack('BBBfff',0x96,0x00,0x01,x,y,z)

def vec_packet(vector):
    x, y, z = vector

    return struct.pack('BBBfff',0x96,0x00,0x03,x,y,z)

def end_packet():

    return struct.pack('BBB',0x96, 0x00, 0x04)

def lat_lon_at(satellite, ti, earth_radius):
    geo = satellite.at(ti)

    lat, lon = wgs84.latlon_of(geo)
    lat, lon = lat.arcminutes()/60, lon.arcminutes()/60
    height = wgs84.height_of(geo).km

    return lat_lon_rotation(lat, lon, np.array([earth_radius + height, 0, 0]).T)

def lat_lon_rotation(lat, lon, vector):
    s = math.sin(lat * math.pi / 180)
    c = math.cos(lat * math.pi / 180)
    dcm_y = np.array([[c, 0, s],
                      [0, 1, 0],
                      [-s, 0, c]])

    s = math.sin(lon * math.pi / 180)
    c = math.cos(lon * math.pi / 180)
    dcm_z = np.array([[c, -s, 0],
                      [s, c, 0],
                      [0, 0, 1]])

    res = (dcm_z @ (dcm_y @ vector))

    return res.T

stations_url = 'https://celestrak.org/NORAD/elements/gp.php?INTDES=2022-072'
satellites = load.tle_file(stations_url, reload=True)
print('Loaded', len(satellites), 'satellites')

satellite = {sat.model.satnum: sat for sat in satellites}[52935]

ts = load.timescale()
t = ts.tt_jd(range(2299159, 2299163))

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('127.0.0.1', 30000))

earth_radius = 6378.1
loc = [np.array([earth_radius*1.3, 0, 0]), np.array([0, 8000, 0])]
quaternion = [0,0,0,1]

client_socket.send(attitude_packet(quaternion))
client_socket.send(location_packet(loc[0]))

for ti in t:
    loc = lat_lon_at(satellite, ti, earth_radius)

    print(loc)
    time.delay(0.1)


client_socket.close()